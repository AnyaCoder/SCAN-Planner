#!/usr/bin/env python3
"""Shared VLM request and low-latency positional stream parsing."""

import base64
import json
import os
import re
import threading
from urllib.request import Request, urlopen


class QueryCancelled(RuntimeError):
    pass


class QueryCancellation:
    """Cancellation token that can close the active streaming response."""

    def __init__(self):
        self._event = threading.Event()
        self._lock = threading.Lock()
        self._close_response = None

    def cancel(self):
        with self._lock:
            close_response = self._close_response
        self._event.set()
        if close_response is not None:
            close_response()

    def is_set(self):
        return self._event.is_set()

    def attach_response(self, response):
        with self._lock:
            self._close_response = response.close
        if self._event.is_set():
            response.close()

    def detach_response(self):
        with self._lock:
            self._close_response = None


def load_dotenv(path):
    if not path or not os.path.isfile(path):
        return
    with open(path, encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            if key.startswith("export "):
                key = key[7:].strip()
            value = value.strip().strip("\"'")
            if key:
                os.environ.setdefault(key, value)


def _message_text(body):
    choice = body["choices"][0]
    message = choice["message"].get("content", "")
    if not message:
        message = choice["message"].get("reasoning_content", "")
    return choice, message


class StreamValueParser:
    """Parse one floor point, top-down request, or signed in-place turn."""

    def __init__(self):
        self.text = ""

    def feed(self, text, final=False):
        self.text += text
        return self.values(final=final)

    def values(self, final=False):
        # JSON is accepted only as a compatibility fallback.
        json_match = re.search(r"\{.*\}", self.text, re.DOTALL)
        if json_match is not None:
            try:
                return json.loads(json_match.group(0))
            except json.JSONDecodeError:
                pass

        cleaned = re.sub(
            r"^\s*```[A-Za-z0-9_-]*\s*",
            "",
            self.text,
        ).strip()
        control_match = re.fullmatch(
            r"(?:(?:view|context)\s*[:=]\s*)?"
            r"(topdown|map)\s*[.!]?",
            cleaned,
            re.IGNORECASE,
        )
        if control_match is not None:
            return {
                "walkable": False,
                "navigation_view": "topdown",
            }

        number = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)"
        point_match = re.search(
            r"(?im)^\s*(?:point|map_point)\s*[:=]\s*"
            r"(" + number + r")[ \t]+(" + number + r")",
            self.text,
        )
        if point_match is not None:
            point_end = point_match.end(2)
            if (
                not final
                and (
                    point_end == len(self.text)
                    or not self.text[point_end].isspace()
                )
            ):
                return {}
            return {
                "walkable": True,
                "map_x": float(point_match.group(1)),
                "map_y": float(point_match.group(2)),
            }
        turn_match = re.search(
            r"(?im)^\s*(?:turn|turn_deg)\s*[:=]\s*"
            r"(" + number + r")\s*(?:deg|degrees?)?",
            self.text,
        )
        if turn_match is not None:
            turn_end = turn_match.end()
            if not final and turn_end == len(self.text):
                return {}
            return {"turn_deg": float(turn_match.group(1))}
        return {}


def parse_decision_values(text):
    result = StreamValueParser().feed(text, final=True)
    if result.get("navigation_view") == "topdown":
        return result
    if result.get("map_x") is not None and result.get("map_y") is not None:
        return result
    if result.get("turn_deg") is not None:
        return result
    raise RuntimeError("VLM decision is missing a point, turn, or topdown request")


def build_chat_messages(
    prompt,
    image_jpegs,
    system_prompt=None,
    history_messages=None,
):
    """Build one chat request with images only in the current user turn."""
    messages = []
    if system_prompt:
        messages.append(
            {
                "role": "system",
                "content": str(system_prompt),
            }
        )
    for message in history_messages or []:
        role = str(message.get("role", "")).strip().lower()
        content = message.get("content")
        if role not in ("user", "assistant") or not isinstance(
            content, str
        ):
            raise ValueError("chat history entries must be text user/assistant messages")
        messages.append({"role": role, "content": content})

    image_jpegs = list(image_jpegs)
    if not image_jpegs:
        raise RuntimeError("VLM request has no images")
    content = [{"type": "text", "text": str(prompt)}]
    content.extend(
        {
            "type": "image_url",
            "image_url": {
                "url": "data:image/jpeg;base64,"
                + base64.b64encode(image_jpeg).decode("ascii")
            },
        }
        for image_jpeg in image_jpegs
    )
    messages.append({"role": "user", "content": content})
    return messages


def query_json(
    args,
    prompt,
    image_jpegs,
    system_prompt=None,
    history_messages=None,
    stream_callback=None,
    cancel_event=None,
):
    api_key = os.environ.get("VLM_API_KEY", "")
    if not api_key:
        raise RuntimeError("VLM_API_KEY is missing")
    payload = {
        "model": args.model,
        "messages": build_chat_messages(
            prompt,
            image_jpegs,
            system_prompt=system_prompt,
            history_messages=history_messages,
        ),
        "max_tokens": int(getattr(args, "max_output_tokens", 768)),
        "temperature": 0,
        "stream": stream_callback is not None,
    }
    reasoning_effort = getattr(args, "reasoning_effort", "low")
    if reasoning_effort in ("low", "medium", "high"):
        payload["reasoning_effort"] = reasoning_effort
    if getattr(args, "disable_thinking", False):
        payload["chat_template_kwargs"] = {"enable_thinking": False}
    request = Request(
        args.api_url,
        data=json.dumps(payload).encode("utf-8"),
        headers={
            "Authorization": "Bearer " + api_key,
            "Content-Type": "application/json",
        },
    )
    if cancel_event is not None and cancel_event.is_set():
        raise QueryCancelled("VLM generation cancelled before request")
    choice = {}
    message = ""
    try:
        with urlopen(request, timeout=args.api_timeout) as response:
            if hasattr(cancel_event, "attach_response"):
                cancel_event.attach_response(response)
            try:
                if cancel_event is not None and cancel_event.is_set():
                    raise QueryCancelled("VLM generation cancelled")
                content_type = response.headers.get("Content-Type", "")
                if (
                    stream_callback is None
                    or "text/event-stream" not in content_type
                ):
                    body = json.load(response)
                    choice, message = _message_text(body)
                    if stream_callback is not None and message:
                        stream_callback(message, "content")
                else:
                    content_parts = []
                    reasoning_parts = []
                    for raw_line in response:
                        if (
                            cancel_event is not None
                            and cancel_event.is_set()
                        ):
                            raise QueryCancelled("VLM generation cancelled")
                        line = raw_line.decode("utf-8", "replace").strip()
                        if not line.startswith("data:"):
                            continue
                        payload = line[5:].strip()
                        if payload == "[DONE]":
                            break
                        if not payload:
                            continue
                        event = json.loads(payload)
                        choice = event["choices"][0]
                        delta = choice.get("delta") or {}
                        reasoning = delta.get("reasoning_content")
                        if reasoning:
                            reasoning_parts.append(reasoning)
                            stream_callback(reasoning, "reasoning")
                        content = delta.get("content")
                        if content:
                            content_parts.append(content)
                            stream_callback(content, "content")
                    message = (
                        "".join(content_parts)
                        or "".join(reasoning_parts)
                    )
            finally:
                if hasattr(cancel_event, "detach_response"):
                    cancel_event.detach_response()
        if stream_callback is not None:
            stream_callback("", "final")
    except QueryCancelled:
        raise
    except (OSError, ValueError) as exc:
        if cancel_event is not None and cancel_event.is_set():
            raise QueryCancelled("VLM generation cancelled") from exc
        raise
    if cancel_event is not None and cancel_event.is_set():
        raise QueryCancelled("VLM generation cancelled")
    try:
        return parse_decision_values(message)
    except Exception as exc:
        raise RuntimeError(
            "VLM value-line parse failed: finish=%s chars=%d: %s"
            % (choice.get("finish_reason"), len(message), exc)
        ) from exc
