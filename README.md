# nynu — OV5640 视觉处理系统

> 硬件平台：**PGL50H6FB484**（Pango Design Suite FPGA）  
> 摄像头：**OV5640**（DVP 接口，640×480 @ 30fps）  

## 工程结构

```
nynu/
├── rtl/
│   ├── top.v                        # 顶层模块（完整处理链路）
│   ├── camera/
│   │   └── ov5640_capture.v         # DVP 接口采集
│   ├── isp/
│   │   ├── rgb2gray.v               # RGB565 → 灰度
│   │   ├── gaussian_filter.v        # 3×3 高斯滤波（降噪）
│   │   ├── adaptive_threshold.v     # 自适应二值化
│   │   ├── sobel_edge.v             # Sobel 边缘检测
│   │   ├── color_detect.v           # HSV 颜色识别
│   │   └── target_detect.v          # 质心计算 & 包围盒
│   ├── uart/
│   │   └── uart_tx.v                # UART 发送（含帧打包）
│   └── ctrl/
│       └── vision_ctrl.v            # 视觉→控制状态机
├── sim/
│   └── tb_image_proc.v              # 端到端仿真 testbench
├── scripts/
│   └── offline_test.py              # PC 端离线验证脚本
└── constraints/
    └── pgl50_ov5640.fdc             # 时序约束 & 引脚分配模板（PDS FDC 格式）
```

## 处理链路

```
OV5640 DVP
    │
    ▼
ov5640_capture          ← PCLK/HREF/VSYNC/D[7:0]，输出 RGB565 像素流
    │
    ├──────────────────── 颜色检测通路（cfg_mode=0）
    │   color_detect     ← RGB565→HSV 范围判定，H/S/V 可运行时配置
    │
    └──────────────────── 边缘/形状通路（cfg_mode=1）
        rgb2gray
        gaussian_filter  ← 3×3 高斯降噪
        sobel_edge       ← |Gx|+|Gy| 梯度检测
        adaptive_threshold ← 自适应二值化
    │
    ▼
target_detect            ← 逐帧累积前景像素，帧结束输出质心/包围盒
    │
    ▼
vision_ctrl              ← 状态机: DETECTING→TRACKING→LOST→FAULT
    │
    ├── uart_frame_tx    ← 11字节帧（0xAA55 + 类型+坐标+面积+校验）
    └── led / pan_ref / tilt_ref
```

## 快速上手

### 1. 离线 PC 验证（先调通算法参数）

```bash
pip install opencv-python numpy
# 颜色检测（摄像头实时）
python scripts/offline_test.py --mode color --h-min 10 --h-max 25 \
    --s-min 150 --v-min 100
# 边缘检测（图片测试）
python scripts/offline_test.py --source test.jpg --mode edge --edge-thr 40
```

**按键**：`Q` 退出，`S` 截图保存

### 2. RTL 仿真（Icarus Verilog）

```bash
iverilog -o sim.vvp \
    rtl/camera/ov5640_capture.v \
    rtl/isp/rgb2gray.v \
    rtl/isp/gaussian_filter.v \
    rtl/isp/adaptive_threshold.v \
    rtl/isp/color_detect.v \
    rtl/isp/sobel_edge.v \
    rtl/isp/target_detect.v \
    rtl/uart/uart_tx.v \
    rtl/ctrl/vision_ctrl.v \
    sim/tb_image_proc.v
vvp sim.vvp
gtkwave tb_image_proc.vcd
```

### 3. FPGA 综合（Pango Design Suite）

1. 新建工程，目标器件选 **PGL50H6FB484**
2. 添加 `rtl/` 下所有 `.v` 文件，顶层设为 `top`
3. 导入 `constraints/pgl50_ov5640.fdc`，**对照原理图修改引脚编号**
4. 综合 → 布局布线 → 生成比特流 → 下载

### 4. 颜色参数调试流程

1. 运行 `offline_test.py --mode color`，对着目标物体
2. 调整 `--h-min/--h-max`（H 范围），`--s-min`（饱和度下限），`--v-min`（亮度下限）
3. 确认质心稳定后，将参数写入 FPGA 顶层的 `cfg_*` 端口（可用拨码开关或软核驱动）

## UART 通信协议

| 字节 | 含义 |
|------|------|
| 0    | `0xAA`（帧头）|
| 1    | `0x55`（帧头）|
| 2    | 类型：`0x01`=检测到目标，`0x00`=无目标 |
| 3    | CX 高 3 bit（`[10:8]`）|
| 4    | CX 低 8 bit |
| 5    | CY 高 2 bit（`[9:8]`）|
| 6    | CY 低 8 bit |
| 7    | AREA 高字节 |
| 8    | AREA 低字节 |
| 9    | FLAGS（状态机状态低 4 bit）|
| 10   | XOR 校验（字节 2~9 的异或）|

波特率：**115200 bps，8N1**

## 状态机说明

| 状态 | LED | 含义 |
|------|-----|------|
| WAIT_CAM  | 灭  | 等待摄像头初始化（等待 30 帧）|
| DETECTING | 蓝  | 搜索目标 |
| TRACKING  | 绿  | 目标锁定，持续输出质心 |
| LOST      | 红  | 目标丢失，等待重新出现 |
| FAULT     | 橙  | 连续 100 帧异常，进入保护模式 |

## 资源估算（PGL50 参考）

| 模块 | LUT | FF | BRAM |
|------|-----|-----|------|
| ov5640_capture | ~50 | ~80 | 0 |
| rgb2gray | ~30 | ~20 | 0 |
| gaussian_filter | ~200 | ~100 | 2（Line Buffer）|
| sobel_edge | ~300 | ~150 | 2（Line Buffer）|
| color_detect | ~500 | ~200 | 0 |
| target_detect | ~100 | ~120 | 0 |
| uart_tx | ~50 | ~40 | 0 |
| vision_ctrl | ~80 | ~60 | 0 |
| **合计** | **~1310** | **~770** | **4** |

PGL50 资源充裕，可进一步扩展多目标检测或 CNN 加速器。

## 注意事项

1. **OV5640 SCCB 初始化**：必须先由 I2C 控制器向 OV5640 写入寄存器配置才能输出图像，官方例程中已包含，本工程直接对接其输出即可。
2. **跨时钟域**：`cam_pclk`（来自 OV5640）与 `sys_clk`（板载晶振）为异步时钟域，约束文件中已声明 `set_clock_groups -asynchronous`。
3. **Line Buffer**：高斯/Sobel 模块内置 2 行 BRAM，分辨率超过 640 时需增大 `IMG_W` 参数并重新评估 BRAM 资源。
4. **引脚分配**：`constraints/pgl50_ov5640.fdc` 中的引脚号为**占位示例**，必须对照实际板子原理图修改后才能综合下载。
