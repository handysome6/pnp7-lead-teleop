"""EVK4 事件流实时预览，用于摆放和对准相机。

事件相机只对亮度变化响应 —— 完全静止的场景不产生任何事件，画面会是全黑。
所以这里用带衰减的累积（time surface）而不是单帧事件图：轻微的振动、气流、
甚至手在视野边缘晃动都会留下逐渐淡出的痕迹，足以看清场景轮廓。

叠加了网格和中心十字，便于判断工作区是否在视野内。

按键:
  q / ESC  退出
  d        切换 衰减累积 / 原始事件帧
  h        切换 热像素标记
  s        保存当前画面
  + / -    调整衰减速度
"""
from __future__ import annotations

import argparse
import os
import time

import cv2
import numpy as np
from metavision_core.event_io import EventsIterator


def load_hot(path):
    if not path or not os.path.exists(path):
        return []
    out = []
    for line in open(path):
        p = line.split()
        if len(p) >= 2:
            out.append((int(p[0]), int(p[1])))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hot-pixels", default="/work/hot_pixels.txt")
    ap.add_argument("--delta-t", type=int, default=20000)
    ap.add_argument("--decay", type=float, default=0.90,
                    help="每帧衰减系数，越小痕迹消失越快")
    ap.add_argument("--scale", type=float, default=0.75, help="显示缩放")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="自动退出时间；0 表示一直运行")
    args = ap.parse_args()

    hot = load_hot(args.hot_pixels)
    it = EventsIterator(input_path="", delta_t=args.delta_t)
    h, w = it.get_size()
    print(f"分辨率 {w}x{h}   屏蔽 {len(hot)} 个热像素")
    print("提示：事件相机只对变化响应，静止场景是黑的。")
    print("      在视野里挥手或移动物体才能看到轮廓。\n")

    win = ("EVK4 event stream  --  q:quit  d:mode  h:hotpix  "
           "s:save  +/-:decay")
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, int(w * args.scale), int(h * args.scale))

    surface = np.zeros((h, w), np.float32)
    decay = args.decay
    show_decay = True
    show_hot = True
    saved = 0
    t_start = time.monotonic()
    n_recent, t_rate, rate = 0, time.monotonic(), 0.0

    for evs in it:
        frame_pos = np.zeros((h, w), np.float32)
        frame_neg = np.zeros((h, w), np.float32)

        if len(evs):
            x, y, p = evs["x"], evs["y"], evs["p"]
            if hot:
                # 只有两三个热像素时，逐个做全数组比较比构造 int64 编码再
                # np.isin 更快（实测 2.3 ms vs 4.7 ms）。
                m = np.ones(len(evs), bool)
                for hx, hy in hot:
                    m &= ~((x == hx) & (y == hy))
                x, y, p = x[m], y[m], p[m]
            n_recent += len(x)
            if len(x):
                # 用 bincount 而非 np.add.at：后者是 unbuffered 散射累加，
                # 实测每批 28.8 ms vs 3.7 ms，是预览卡顿的主因。
                pos = p == 1
                idx = y.astype(np.int64) * w + x
                frame_pos = np.bincount(
                    idx[pos], minlength=h * w).astype(np.float32).reshape(h, w)
                frame_neg = np.bincount(
                    idx[~pos], minlength=h * w).astype(np.float32).reshape(h, w)

        now = time.monotonic()
        if now - t_rate >= 0.5:
            rate = n_recent / (now - t_rate) / 1e6
            n_recent, t_rate = 0, now

        if show_decay:
            surface *= decay
            surface += frame_pos + frame_neg
            vis = np.clip(surface * 40, 0, 255).astype(np.uint8)
            img = cv2.applyColorMap(vis, cv2.COLORMAP_BONE)
        else:
            img = np.zeros((h, w, 3), np.uint8)
            img[..., 2] = np.clip(frame_pos * 90, 0, 255)   # 正极性 -> 红
            img[..., 0] = np.clip(frame_neg * 90, 0, 255)   # 负极性 -> 蓝

        # 网格与中心十字，辅助对准
        for gx in range(1, 4):
            cv2.line(img, (w * gx // 4, 0), (w * gx // 4, h), (45, 45, 45), 1)
        for gy in range(1, 3):
            cv2.line(img, (0, h * gy // 3), (w, h * gy // 3), (45, 45, 45), 1)
        cx, cy = w // 2, h // 2
        cv2.line(img, (cx - 22, cy), (cx + 22, cy), (0, 200, 200), 1)
        cv2.line(img, (cx, cy - 22), (cx, cy + 22), (0, 200, 200), 1)

        if show_hot:
            for hx, hy in hot:
                cv2.circle(img, (hx, hy), 11, (0, 165, 255), 1)

        cv2.rectangle(img, (0, 0), (w, 30), (0, 0, 0), -1)
        # cv2.putText 只有 Hershey 字体，中文会渲染成 ?，这里用英文
        mode = f"decay {decay:.2f}" if show_decay else "raw events"
        hot_txt = f"hotpix:{len(hot)}" if show_hot else "hotpix:off"
        cv2.putText(img, f"{rate:5.2f} Mev/s   {mode}   {hot_txt}   {w}x{h}",
                    (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow(win, img)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        if k == ord("d"):
            show_decay = not show_decay
            surface[:] = 0
        if k == ord("h"):
            show_hot = not show_hot
        if k == ord("s"):
            saved += 1
            p = f"/work/event_view_{saved:02d}.png"
            cv2.imwrite(p, img)
            print(f"已保存 {p}")
        if k == ord("+") or k == ord("="):
            decay = min(0.99, decay + 0.01)
        if k == ord("-"):
            decay = max(0.50, decay - 0.01)

        if args.seconds and time.monotonic() - t_start > args.seconds:
            break

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
