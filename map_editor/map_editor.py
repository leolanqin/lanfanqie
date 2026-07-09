#!/usr/bin/env python3
"""交互式 PGM 地图编辑器（ROS 导航地图专用）
依赖：pip3 install opencv-python numpy matplotlib（均已安装）
操作说明：
  左键拖拽        - 画障碍物（黑色，像素值 0）
  右键拖拽        - 画空闲区（白色，像素值 254）
  中键拖拽        - 画未知区域（灰色，像素值 205）
  Space+中       键拖拽  - 平移视图
  滚轮            - 以鼠标为中心缩放
  [ / ]           - 缩小/增大画笔
  Ctrl+Z          - 撤销
  Ctrl+S          - 保存
  R               - 重置视图
  Q 或者 ESC       - 退出
  Ctrl + 左键点击   -输出当前点：像素坐标 (px, py) ROS 世界坐标 (wx, wy)（单位：米）在图上显示红色标记点 + 坐标文本
"""
 
import sys
import os
 
# 抑制 Jetson 上 libEGL 的 DRI 认证噪音（非致命，不影响功能）
_stderr_fd = os.dup(2)
_stderr_tmp = os.open(os.devnull, os.O_WRONLY)
os.dup2(_stderr_tmp, 2)
os.close(_stderr_tmp)
 
import cv2
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
# 禁用 matplotlib 默认快捷键，避免跟我们的 Ctrl+S / Q 冲突
matplotlib.rcParams["keymap.save"] = ""
matplotlib.rcParams["keymap.quit"] = ""
matplotlib.rcParams["keymap.quit_all"] = ""
matplotlib.rcParams["keymap.fullscreen"] = ""
matplotlib.rcParams["keymap.home"] = ""
matplotlib.rcParams["keymap.back"] = ""
matplotlib.rcParams["keymap.forward"] = ""
matplotlib.rcParams["keymap.pan"] = ""
matplotlib.rcParams["keymap.zoom"] = ""
matplotlib.rcParams["keymap.grid"] = ""
import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="matplotlib")
warnings.filterwarnings("ignore", category=UserWarning, module="PIL")
warnings.filterwarnings("ignore", category=UserWarning, module="tkinter")
import tkinter
from matplotlib import pyplot as plt
from matplotlib.backend_bases import MouseButton
 
# 恢复 stderr
os.dup2(_stderr_fd, 2)
os.close(_stderr_fd)
 
# 像素值定义（ROS 约定）
OCCUPIED = 0    # 障碍物（黑色）
FREE = 254      # 空闲（白色）
UNKNOWN = 205   # 未知（灰色）
 
DEFAULT_BRUSH = 3
MIN_BRUSH = 1
MAX_BRUSH = 50
 
 
class MapEditor:
    def __init__(self, filepath):
        self.filepath = filepath
        self.img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
        if self.img is None:
            print(f"错误：无法读取 {filepath}")
            sys.exit(1)
        self.h, self.w = self.img.shape
        self.history = [self.img.copy()]
        self.brush_radius = DEFAULT_BRUSH

        # 目标地图的左下角ros世界坐标
        self.resolution = 0.05
        self.origin = (-16, -11)

        print("DEBUG origin =", self.origin)
        print("DEBUG resolution =", self.resolution)
 
        # 绘制状态
        self.drawing = False
        self.mode = None       # 'occupy', 'free', 'unknown'
        self._last_x = None    # Bresenham 插值：上一帧鼠标位置
        self._last_y = None
 
        # 平移状态
        self._space_held = False
        self._panning = False
        self._pan_start_xlim = None
        self._pan_start_ylim = None
        self._pan_mouse_start = None
 
        # 创建界面（关闭默认 toolbar，避免快捷键冲突）
        self.fig, self.ax = plt.subplots(figsize=(14, 8))
        self.fig.canvas.manager.toolbar.pack_forget()  # 隐藏工具栏
        self.fig.canvas.manager.set_window_title(f"Map Editor - {os.path.basename(filepath)}")
        self.im = self.ax.imshow(self.img, cmap="gray", vmin=0, vmax=255, interpolation="none")
        self._update_title()
 
        # 底部信息栏
        unique, counts = np.unique(self.img, return_counts=True)
        info = ", ".join(f"{v}={c/self.img.size*100:.1f}%" for v, c in zip(unique, counts))
        self._info_text = self.fig.text(0.01, 0.01,
                                        f"Size: {self.w}x{self.h} | {info}", fontsize=8, family="monospace")
 
        # 事件绑定
        self.fig.canvas.mpl_connect("button_press_event", self._on_press)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_move)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key_press)
        self.fig.canvas.mpl_connect("key_release_event", self._on_key_release)
 
        print(f"Map Editor ready: {self.w}x{self.h}  [{info}]")
        print("L=Occupy | R=Free | M=Unknown | Wheel=Zoom | Space+Drag=Pan | []=Brush | Ctrl+S/Z | R=Reset | Q=Quit")
        plt.show()
 
    # ─── 绘制核心 ───────────────────────────────────────
 
    def _set_pixel(self, x, y):
        """设置单个像素（带笔刷范围）"""
        val = {
            "occupy": OCCUPIED,
            "free": FREE,
            "unknown": UNKNOWN
        }[self.mode]

        cv2.circle(
            self.img,
            (int(x), int(y)),
            self.brush_radius,
            int(val),
            thickness=-1
        )
 
    @staticmethod
    def _bresenham_line(x0, y0, x1, y1):
        """Bresenham 直线算法：返回两点之间所有整数像素坐标"""
        points = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                if x0 == x1:
                    break
                err += dy
                x0 += sx
            if e2 <= dx:
                if y0 == y1:
                    break
                err += dx
                y0 += sy
        return points
 
    def _draw_stroke(self, x, y):
        """从上一帧位置到当前位置，走 Bresenham 插值落笔"""
        if x is None or y is None:
            return
        if self._last_x is not None and self._last_y is not None:
            for px, py in self._bresenham_line(self._last_x, self._last_y, int(x), int(y)):
                self._set_pixel(px, py)
        else:
            self._set_pixel(x, y)
        self._last_x, self._last_y = int(x), int(y)
        self.im.set_data(self.img)
        self.fig.canvas.draw_idle()

    # ─── 获取坐标 ───────────────────────────────────────

    def _pixel_to_world(self, x, y):
        ox, oy = self.origin
        r = self.resolution

        wx = ox + x * r
        wy = oy + (self.h - y) * r

        return wx, wy
    
    # ─── 在地图上显示项像素坐标 ───────────────────────────────────────
    def _show_coordinate(self, x, y):
        """在图上显示一个红色坐标标记"""

        # 删除旧标记
        if hasattr(self, "_coord_marker") and self._coord_marker is not None:
            self._coord_marker.remove()

        if hasattr(self, "_coord_label") and self._coord_label is not None:
            self._coord_label.remove()

        # 画红色十字
        self._coord_marker = self.ax.plot(
            x,
            y,
            marker="+",
            color="red",
            markersize=12,
            markeredgewidth=2
        )[0]

        # 显示文字
        self._coord_label = self.ax.text(
            x + 3,
            y + 3,
            f"({int(x)}, {int(y)})",
            color="red",
            fontsize=9,
            bbox=dict(facecolor="white", alpha=0.7, edgecolor="none")
        )

        self.fig.canvas.draw_idle()
 
    # ─── 鼠标事件 ───────────────────────────────────────
 
    def _on_press(self, event):
        if event.inaxes != self.ax:
            return

        # -------------------------
        # Ctrl + 左键：查询坐标（最高优先级）
        # -------------------------
        if (event.key == "control"
                and event.button == MouseButton.LEFT
                and event.xdata is not None
                and event.ydata is not None):

            px = int(event.xdata)
            py = int(event.ydata)

            wx, wy = self._pixel_to_world(px, py)
            pixel = int(self.img[py, px])

            print(f"ROS世界坐标: ({wx:.3f}, {wy:.3f}) m | pixel={pixel}")

            self._show_coordinate(px, py)

            return  # ❗关键：必须提前 return

        # -------------------------
        # Space + 中键：平移
        # -------------------------
        if self._space_held and event.button == MouseButton.MIDDLE:
            self._panning = True
            self._pan_start_xlim = self.ax.get_xlim()
            self._pan_start_ylim = self.ax.get_ylim()
            self._pan_mouse_start = (event.x, event.y)
            return

        # -------------------------
        # 普通绘制
        # -------------------------
        self.drawing = True
        self._last_x, self._last_y = None, None

        if event.button == MouseButton.LEFT:
            self.mode = "occupy"

        elif event.button == MouseButton.RIGHT:
            self.mode = "free"

        elif event.button == MouseButton.MIDDLE:
            # 如果按着 Space，则已经进入平移模式
            if self._space_held:
                return
            self.mode = "unknown"

        else:
            return

        self._draw_stroke(event.xdata, event.ydata)
 
    def _on_move(self, event):
        if event.inaxes != self.ax:
            self._last_x = None
            self._last_y = None
            self.drawing = False
            return

        if self._panning:
            inv = self.ax.transData.inverted()

            start = inv.transform(self._pan_mouse_start)
            now = inv.transform((event.x, event.y))

            dx = now[0] - start[0]
            dy = now[1] - start[1]

            self.ax.set_xlim(
                self._pan_start_xlim[0] - dx,
                self._pan_start_xlim[1] - dx
            )

            self.ax.set_ylim(
                self._pan_start_ylim[0] - dy,
                self._pan_start_ylim[1] - dy
            )

            self.fig.canvas.draw_idle()
            return

        if self.drawing:
            if event.xdata is None or event.ydata is None:
                return
            self._draw_stroke(event.xdata, event.ydata)
    
    def _on_release(self, event):

        # 结束平移
        if self._panning:
            self._panning = False

            # 防止 Matplotlib 丢失 Space 的 key_release 事件
            self._space_held = False

            return

        # 结束绘制
        if self.drawing:
            self.drawing = False
            self._last_x = None
            self._last_y = None
            self.mode = None

            self.history.append(self.img.copy())
            if len(self.history) > 50:
                self.history.pop(0)
 
    def _on_scroll(self, event):
        """滚轮缩放（以鼠标所在像素为中心）"""
        if event.inaxes != self.ax:
            return
        scale = 0.65 if event.button == "up" else 1.55  # 上滚放大，下滚缩小
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        cx, cy = event.xdata, event.ydata
        if cx is None or cy is None:
            return
        new_w = (xlim[1] - xlim[0]) * scale
        new_h = (ylim[0] - ylim[1]) * scale  # y 轴反转
 
        # 保持鼠标位置不动
        rx = (cx - xlim[0]) / (xlim[1] - xlim[0])
        ry = (cy - ylim[1]) / (ylim[0] - ylim[1])
 
        self.ax.set_xlim(cx - new_w * rx, cx + new_w * (1 - rx))
        self.ax.set_ylim(cy + new_h * (1 - ry), cy - new_h * ry)
        self.fig.canvas.draw_idle()

     # ─── 安全退出 ───────────────────────────────────────
    def _safe_exit(self):
        print("正在安全退出 Map Editor...")

        try:
            # 关闭 matplotlib 所有窗口
            plt.close("all")

            # 清理 figure
            if hasattr(self, "fig"):
                self.fig.clf()
                plt.close(self.fig)

            # 强制断开 tkinter 主循环（关键）
            import tkinter as tk
            root = tk._default_root
            if root is not None:
                root.quit()
                root.destroy()

        except Exception as e:
            print("退出清理异常:", e)

        finally:
            print("Map Editor 已退出")
            os._exit(0)  
    
    # ─── 键盘事件 ───────────────────────────────────────
    
    def _on_key_press(self, event):
        if event.key == " ":
            self._space_held = True
        elif event.key == "ctrl+z":
            self._undo()
        elif event.key == "ctrl+s":
            self._save()
        elif event.key in ("q", "escape"):
            self._safe_exit()
        elif event.key == "r":
            self.ax.set_xlim(0, self.w)
            self.ax.set_ylim(self.h, 0)
            self.fig.canvas.draw_idle()
        elif event.key == "[":
            self.brush_radius = max(MIN_BRUSH, self.brush_radius - 1)
            self._update_title()
            print(f"画笔半径: {self.brush_radius}")
        elif event.key == "]":
            self.brush_radius = min(MAX_BRUSH, self.brush_radius + 1)
            self._update_title()
            print(f"画笔半径: {self.brush_radius}")
 
    def _on_key_release(self, event):
        if event.key == " ":
            self._space_held = False
 
    # ─── 标题栏 ─────────────────────────────────────────
 
    def _update_title(self):
        mode_names = {"occupy": "Occupy(0)", "free": "Free(254)", "unknown": "Unknown(205)"}
        self.ax.set_title(
            f"Brush:{self.brush_radius}px | "
            f"L:Occupy | R:Free | M:Unknown | "
            f"Wheel:Zoom | Space+Drag:Pan | Ctrl+S/Z",
            fontsize=9)
 
    # ─── 撤销 / 保存 ────────────────────────────────────
 
    def _undo(self):
        if len(self.history) > 1:
            self.history.pop()
            self.img = self.history[-1].copy()
            self.im.set_data(self.img)
            self.fig.canvas.draw_idle()
            print(f"撤销 ({len(self.history)-1} 步历史)")
 
    def _save(self):
        cv2.imwrite(self.filepath, self.img)
        unique, counts = np.unique(self.img, return_counts=True)
        info = ", ".join(f"{v}={c/self.img.size*100:.1f}%" for v, c in zip(unique, counts))
        self._info_text.set_text(f"Size: {self.w}x{self.h} | {info}")
        self.fig.canvas.draw_idle()
        print(f"已保存到 {self.filepath}  [{info}]")
 
 
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()

    parser.add_argument("map", nargs="?", default="test5.pgm", help="map file path")

    args = parser.parse_args()

    path = args.map

    if not path.endswith((".pgm", ".png", ".bmp")):
        print("用法: python3 map_editor.py <map>")
        sys.exit(1)

    MapEditor(
        filepath=path,
    )