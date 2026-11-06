# Go2 语义感知试验

## 相机

Go2 前置相机由现有只读服务提供：

```text
Go2 本机:      http://127.0.0.1:8765/frame.jpg
工作站访问:    http://192.168.76.1:8765/frame.jpg
```

实际输出为 `640x360`、约 `5 Hz` 的 JPEG。服务只调用 `VideoClient.GetImageSample()`，不涉及运动接口。

## 模型结论

| 模型 | Orin NX 实测 | 用途 |
|---|---:|---|
| YOLOv8n FP16 TensorRT | 推理 `10.2 ms`，端到端 `20-27 ms` | 主检测器，5 Hz 实时跑 |
| YOLO11n FP16 TensorRT | 推理 `12.4 ms`，端到端 `22-29 ms` | 备选主检测器，官方新版权重 |
| MobileCLIP2-S0 FP16 TensorRT | 预处理+推理 `19.4 ms` | 主场景语义模型 |
| CLIP ViT-B/32 FP16 TensorRT | 预处理+推理 `16.1 ms` | 第二意见，交叉复核 |
| MobileCLIP2-S0 CPU | 256x256 单图约 `0.41 s` | 已移除，仅作为精度基线 |
| DINOv2-S | 224x224 单图约 `0.32 s` (CPU)，输出 `257x384` 特征 | 更适合做地点识别/检索，不直接给物体标签 |

COCO 80 类不包含门、电梯、楼梯、玻璃墙等导航最需要的类别。当前 YOLO 会出现把白色隔断判成 `refrigerator` 这类误检，CLIP 能参与复核，但不能替代室内语义检测器。

同一帧上，MobileCLIP2 的 TensorRT 输出与 CPU 基线排序完全一致：

```text
CPU:        office 0.826, corridor 0.039, office chair 0.035
TensorRT:   office 0.817, corridor 0.041, office chair 0.033
```

差异来自 FP16 和 OpenCV/PIL 的缩放实现，不影响语义结论。TensorRT 推理比
CPU 快约 21 倍，已经可以按相机帧率持续运行。

## 运行

在 Go2 上：

```bash
cd /home/unitree/SCAN-Planner/perception/yolo_trt
./build/yolo_trt --engine /home/unitree/SCAN-Planner/perception/engines/yolov8n_fp16.engine --frames 10
```

输出：

```text
/home/unitree/SCAN-Planner/perception/data/latest_detected.jpg
/home/unitree/SCAN-Planner/perception/data/latest_detections.json
```

首帧包含 TensorRT warmup，约 `1.2 s`；持续运行时按 5 Hz 更新。

按需跑一次 CLIP：

```bash
cd /home/unitree/SCAN-Planner/perception/clip_trt
./build/clip_trt \
  --model mobileclip \
  --engine /home/unitree/SCAN-Planner/perception/engines/mobileclip2_s0_fp16.engine \
  --image http://127.0.0.1:8765/frame.jpg \
  --labels /home/unitree/SCAN-Planner/perception/models/mobileclip2_s0/labels.txt \
  --embeddings /home/unitree/SCAN-Planner/perception/models/mobileclip2_s0/text_embeddings.bin \
  --bench 50
```

标准 CLIP：

```bash
./build/clip_trt \
  --model clip \
  --engine /home/unitree/SCAN-Planner/perception/engines/clip_vit_b32_fp16.engine \
  --image http://127.0.0.1:8765/frame.jpg \
  --labels /home/unitree/SCAN-Planner/perception/models/clip_vit_b32/labels.txt \
  --embeddings /home/unitree/SCAN-Planner/perception/models/clip_vit_b32/text_embeddings.bin \
  --bench 50
```

`--bench` 会先预热一次，再在进程内重复推理。引擎反序列化约需 `1.9-2.2 s`，
首次推理还要约 `1.3 s` 唤醒 GPU；两者都只发生在进程启动时。实际服务应常驻，
不能每帧拉起一个进程，否则单次快照仍要 `3-4 s`。

一次跑完 YOLO 和 CLIP 快照：

```bash
cd /home/unitree/SCAN-Planner/perception
./run_snapshot.sh /home/unitree/SCAN-Planner/perception/engines/yolov8n_fp16.engine mobileclip
./run_snapshot.sh /home/unitree/SCAN-Planner/perception/engines/yolov8n_fp16.engine clip
```

不传第二个参数时默认使用 MobileCLIP2 TensorRT。

## 构建

模型文件不放进 Git。ONNX 来自 Ultralytics 官方权重导出，使用 `opset=12`；TensorRT engine 必须在本机生成：

```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=/home/unitree/SCAN-Planner/perception/models/yolov8n.onnx \
  --saveEngine=/home/unitree/SCAN-Planner/perception/engines/yolov8n_fp16.engine \
  --fp16
```

首次构建在 25W 模式下约需 10-20 分钟。

CLIP engine 的 ONNX 需固定 batch 和图像尺寸：

```text
MobileCLIP2-S0: [1,3,256,256] -> [1,512], opset 14
CLIP ViT-B/32:  [1,3,224,224] -> [1,512]
```

MobileCLIP2 导出前要对 visual tower 做
`timm.utils.reparameterize_model`。opset 12 不支持
`scaled_dot_product_attention`，所以 MobileCLIP2 使用 opset 14。

重建 CLIP engine：

```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=/home/unitree/SCAN-Planner/perception/models/mobileclip2_s0/visual_fixed.onnx \
  --saveEngine=/home/unitree/SCAN-Planner/perception/engines/mobileclip2_s0_fp16.engine \
  --fp16

/usr/src/tensorrt/bin/trtexec \
  --onnx=/home/unitree/SCAN-Planner/perception/models/clip_vit_b32/vision_fixed.onnx \
  --saveEngine=/home/unitree/SCAN-Planner/perception/engines/clip_vit_b32_fp16.engine \
  --fp16
```

## TensorRT 与 PyTorch 运行环境

YOLO 和 CLIP 的 TensorRT 路径不需要 PyTorch 或 ONNX Runtime GPU。Go2
上的 PyTorch 只用于 Qwen3-VL，安装在独立目录
`/home/unitree/SCAN-Planner/perception/vlm_py`，不进入系统 Python。
JetPack 5 使用的是 NVIDIA 提供的 cp38 aarch64 wheel；ONNX Runtime GPU
没有可直接安装的 cp38 aarch64 PyPI 包。

## Qwen3-VL-2B torch 路线

Go2 上已经部署 `Qwen3-VL-2B-Instruct`，用于低频场景理解，不参与 5 Hz
检测流程。模型放在：

```text
/home/unitree/SCAN-Planner/perception/models/qwen3_vl_2b
```

本地 Python 环境放在：

```text
/home/unitree/SCAN-Planner/perception/vlm_py
```

运行一次相机快照问答：

```bash
PYTHONPATH=/home/unitree/SCAN-Planner/perception/vlm_py \
python3 /home/unitree/SCAN-Planner/perception/qwen3_vl_snapshot.py
```

连续测两次，第二次是热推理：

```bash
PYTHONPATH=/home/unitree/SCAN-Planner/perception/vlm_py \
python3 /home/unitree/SCAN-Planner/perception/qwen3_vl_snapshot.py --runs 2
```

输出 JSON：

```bash
PYTHONPATH=/home/unitree/SCAN-Planner/perception/vlm_py \
python3 /home/unitree/SCAN-Planner/perception/qwen3_vl_snapshot.py --json
```

Orin NX 16GB 实测，640x360 相机帧、fp16、SDPA：

| 项目 | 实测 |
|---|---:|
| 模型加载 | `9.8 s` |
| 图像预处理 | `0.08 s` |
| 首次生成，31 tokens | `8.0 s` |
| 热生成，31 tokens | `4.8 s` |
| 生成速度 | 约 `6.4 tok/s` |
| 峰值显存 | `4.35 GB` |

首次生成包含 CUDA kernel 初始化，服务化时应保持进程常驻并预热一次。
该模型适合“到门口看看有没有人”“前方是否可通行”这类按需查询，不适合
逐帧运行。

安装 torch 时使用 JetPack 5.1.2 对应的 NVIDIA wheel：

```text
torch 2.1.0a0+41361538.nv23.06
torchvision 0.16.0
transformers 4.57.1
```

torchvision 的 `image.so` 会提示 ABI 警告，但这里只用
`torchvision.transforms.v2.functional` 做预处理，不影响图像问答。
