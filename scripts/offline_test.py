#!/usr/bin/env python3
# ============================================================
# 离线测试脚本：offline_test.py
# 功能：在 PC 上验证 ISP 算法（Python 模拟 RTL 逻辑）
#   - 读取图片或摄像头帧
#   - 执行颜色检测、边缘检测、质心计算
#   - 输出结果可视化 + 性能统计
# 依赖：pip install opencv-python numpy
# ============================================================
import cv2
import numpy as np
import time
import argparse
import struct

# ============================================================
# 配置参数（对应 RTL cfg_* 寄存器）
# ============================================================
class Config:
    # 颜色检测 HSV 范围（OpenCV H:0~179, S/V:0~255）
    H_MIN, H_MAX = 10,  25   # 橙色示例
    S_MIN, S_MAX = 150, 255
    V_MIN, V_MAX = 100, 255

    # 二值化参数
    BIAS       = 10           # 自适应阈值偏置
    EDGE_THR   = 50           # Sobel 边缘阈值
    GLOBAL_THR = 128

    # 目标面积过滤
    MIN_AREA   = 200
    MAX_AREA   = 60000

    # 分辨率（对应 FPGA 端参数）
    IMG_W      = 640
    IMG_H      = 480

    # 检测模式：'color' 或 'edge'
    MODE       = 'color'

# ============================================================
# 1. RGB → 灰度（对齐 RTL 实现：76R+150G+29B）
# ============================================================
def rgb2gray_rtl(bgr):
    b, g, r = bgr[:,:,0].astype(np.uint32), bgr[:,:,1].astype(np.uint32), bgr[:,:,2].astype(np.uint32)
    gray = (76*r + 150*g + 29*b) >> 8
    return gray.astype(np.uint8)

# ============================================================
# 2. 高斯滤波（3×3，对应 RTL gaussian_filter）
# ============================================================
def gaussian_filter_rtl(gray):
    kernel = np.array([[1, 2, 1],
                       [2, 4, 2],
                       [1, 2, 1]], dtype=np.float32) / 16.0
    return cv2.filter2D(gray, -1, kernel)

# ============================================================
# 3. 颜色检测（HSV 范围，对应 RTL color_detect）
# ============================================================
def color_detect(bgr, cfg: Config):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # 处理 H 跨零情况（红色系）
    if cfg.H_MIN > cfg.H_MAX:
        mask1 = cv2.inRange(hsv, (cfg.H_MIN, cfg.S_MIN, cfg.V_MIN),
                                  (179,       cfg.S_MAX, cfg.V_MAX))
        mask2 = cv2.inRange(hsv, (0,          cfg.S_MIN, cfg.V_MIN),
                                  (cfg.H_MAX,  cfg.S_MAX, cfg.V_MAX))
        mask = cv2.bitwise_or(mask1, mask2)
    else:
        mask = cv2.inRange(hsv,
                           (cfg.H_MIN, cfg.S_MIN, cfg.V_MIN),
                           (cfg.H_MAX, cfg.S_MAX, cfg.V_MAX))
    return mask

# ============================================================
# 4. 边缘检测（Sobel，对应 RTL sobel_edge）
# ============================================================
def sobel_detect(gray, threshold):
    gx = cv2.Sobel(gray, cv2.CV_16S, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_16S, 0, 1, ksize=3)
    grad = np.abs(gx) + np.abs(gy)
    grad = np.clip(grad >> 2, 0, 255).astype(np.uint8)
    _, edge = cv2.threshold(grad, threshold, 255, cv2.THRESH_BINARY)
    return edge

# ============================================================
# 5. 自适应二值化（对应 RTL adaptive_threshold）
# ============================================================
def adaptive_threshold_rtl(gray, bias):
    # 7×7 均值自适应（近似 RTL 的 7 点行均值）
    binary = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
        cv2.THRESH_BINARY, 7, -bias
    )
    return binary

# ============================================================
# 6. 目标检测与质心计算（对应 RTL target_detect）
# ============================================================
def target_detect(mask, cfg: Config):
    # 形态学去噪
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    clean  = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    clean  = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_area = 0
    for c in contours:
        area = cv2.contourArea(c)
        if cfg.MIN_AREA <= area <= cfg.MAX_AREA and area > best_area:
            best      = c
            best_area = area

    if best is None:
        return None, clean

    M  = cv2.moments(best)
    cx = int(M['m10'] / M['m00']) if M['m00'] != 0 else 0
    cy = int(M['m01'] / M['m00']) if M['m00'] != 0 else 0
    x, y, w, h = cv2.boundingRect(best)
    return {
        'cx': cx, 'cy': cy,
        'area': int(best_area),
        'bbox': (x, y, w, h)
    }, clean

# ============================================================
# 7. UART 帧打包（对应 RTL uart_frame_tx 协议）
# ============================================================
def pack_frame(target_valid, cx=0, cy=0, area=0, flags=0):
    t   = 0x01 if target_valid else 0x00
    cx  = cx  & 0x7FF
    cy  = cy  & 0x3FF
    area= min(area, 0xFFFF)
    payload = bytes([t,
                     (cx >> 8) & 0x07, cx & 0xFF,
                     (cy >> 8) & 0x03, cy & 0xFF,
                     (area >> 8) & 0xFF, area & 0xFF,
                     flags & 0xFF])
    chk = 0
    for b in payload:
        chk ^= b
    return bytes([0xAA, 0x55]) + payload + bytes([chk])

# ============================================================
# 8. 可视化叠加
# ============================================================
def draw_result(frame, result, mode):
    vis = frame.copy()
    h, w = vis.shape[:2]
    # 十字中心线
    cv2.line(vis, (w//2, 0), (w//2, h), (128, 128, 128), 1)
    cv2.line(vis, (0, h//2), (w, h//2), (128, 128, 128), 1)

    if result is not None:
        cx, cy = result['cx'], result['cy']
        bx, by, bw, bh = result['bbox']
        cv2.rectangle(vis, (bx, by), (bx+bw, by+bh), (0, 255, 0), 2)
        cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)
        cv2.line(vis, (w//2, h//2), (cx, cy), (255, 0, 0), 2)
        # 误差标注
        err_x = cx - w//2
        err_y = cy - h//2
        cv2.putText(vis, f"CX:{cx} CY:{cy}  ERR:({err_x},{err_y})  AREA:{result['area']}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(vis, "TARGET FOUND", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else:
        cv2.putText(vis, "NO TARGET", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    cv2.putText(vis, f"MODE:{mode.upper()}", (10, h-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    return vis

# ============================================================
# 9. 主处理循环
# ============================================================
def process_frame(bgr, cfg: Config):
    t0 = time.time()

    if cfg.MODE == 'color':
        mask = color_detect(bgr, cfg)
    else:
        gray   = rgb2gray_rtl(bgr)
        gray   = gaussian_filter_rtl(gray)
        if cfg.MODE == 'edge':
            mask = sobel_detect(gray, cfg.EDGE_THR)
        else:
            mask = adaptive_threshold_rtl(gray, cfg.BIAS)

    result, clean_mask = target_detect(mask, cfg)
    t1 = time.time()

    # 打包 UART 帧
    if result:
        frame_bytes = pack_frame(True, result['cx'], result['cy'], result['area'])
    else:
        frame_bytes = pack_frame(False)

    return result, clean_mask, frame_bytes, (t1 - t0) * 1000  # ms

# ============================================================
# 10. 入口
# ============================================================
def main():
    parser = argparse.ArgumentParser(description='OV5640 视觉离线测试工具')
    parser.add_argument('--source',   default='0',       help='视频源 (0=摄像头, 或图片/视频路径)')
    parser.add_argument('--mode',     default='color',   choices=['color','edge','bin'], help='检测模式')
    parser.add_argument('--h-min',    type=int, default=10)
    parser.add_argument('--h-max',    type=int, default=25)
    parser.add_argument('--s-min',    type=int, default=150)
    parser.add_argument('--s-max',    type=int, default=255)
    parser.add_argument('--v-min',    type=int, default=100)
    parser.add_argument('--v-max',    type=int, default=255)
    parser.add_argument('--min-area', type=int, default=200)
    parser.add_argument('--max-area', type=int, default=60000)
    parser.add_argument('--bias',     type=int, default=10)
    parser.add_argument('--edge-thr', type=int, default=50)
    parser.add_argument('--save',     action='store_true', help='保存输出视频')
    args = parser.parse_args()

    cfg = Config()
    cfg.MODE     = args.mode
    cfg.H_MIN    = args.h_min;    cfg.H_MAX    = args.h_max
    cfg.S_MIN    = args.s_min;    cfg.S_MAX    = args.s_max
    cfg.V_MIN    = args.v_min;    cfg.V_MAX    = args.v_max
    cfg.MIN_AREA = args.min_area; cfg.MAX_AREA = args.max_area
    cfg.BIAS     = args.bias;     cfg.EDGE_THR = args.edge_thr

    # 打开源
    src = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        print(f"[ERROR] 无法打开视频源: {args.source}")
        return

    out_writer = None
    if args.save:
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out_writer = cv2.VideoWriter('output.avi', fourcc, 30.0, (cfg.IMG_W, cfg.IMG_H))

    frame_cnt  = 0
    detect_cnt = 0
    total_ms   = 0.0

    print(f"[INFO] 启动检测，模式={cfg.MODE.upper()}，按 Q 退出")
    print(f"[INFO] 颜色范围: H[{cfg.H_MIN},{cfg.H_MAX}] S[{cfg.S_MIN},{cfg.S_MAX}] V[{cfg.V_MIN},{cfg.V_MAX}]")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.resize(frame, (cfg.IMG_W, cfg.IMG_H))
        result, mask, uart_bytes, proc_ms = process_frame(frame, cfg)

        frame_cnt += 1
        total_ms  += proc_ms
        if result:
            detect_cnt += 1

        # 控制台输出（每 30 帧）
        if frame_cnt % 30 == 0:
            fps = 1000.0 / (total_ms / frame_cnt) if total_ms > 0 else 0
            rate = detect_cnt / frame_cnt * 100
            print(f"[Frame {frame_cnt:5d}] 检测率={rate:.1f}%  avg={total_ms/frame_cnt:.1f}ms  fps={fps:.1f}")
            if result:
                print(f"           目标: cx={result['cx']} cy={result['cy']} area={result['area']}")
            print(f"           UART帧: {uart_bytes.hex(' ')}")

        vis = draw_result(frame, result, cfg.MODE)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        display  = np.hstack([vis, mask_bgr])
        cv2.imshow('OV5640 Vision Test', display)

        if out_writer:
            out_writer.write(vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite(f'snapshot_{frame_cnt}.jpg', vis)
            print(f"[INFO] 截图保存: snapshot_{frame_cnt}.jpg")

    cap.release()
    if out_writer:
        out_writer.release()
    cv2.destroyAllWindows()

    print(f"\n[汇总] 总帧数={frame_cnt}  检测帧={detect_cnt}  "
          f"检测率={detect_cnt/max(frame_cnt,1)*100:.1f}%  "
          f"平均处理时间={total_ms/max(frame_cnt,1):.2f}ms")

if __name__ == '__main__':
    main()
