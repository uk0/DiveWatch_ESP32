"""DiveWatch v5 外壳 - 1.54 寸 / 自定义硬件布局.

坐标系:
    +X = 右侧 (压力传感器在 +X, 长侧面)
    -X = 左侧 (3 按钮在 -X 长侧面)
    +Y = 顶端 (远离 USB-C)
    -Y = 底端 (USB-C 在 -Y 端面)
    +Z = 顶面 (屏幕方向)
    -Z = 底面 (内腔在 +Z 主体内)

外壳分两件:
    bottom = 主体 (下底 + 4 侧壁, 开口在 +Z)
    top    = 顶盖 (含屏幕窗口 + PCB 沉孔)
两件 4 角 M2 螺丝合拢, 上下盖间 1mm O 圈密封槽.
"""

from build123d import (
    BuildPart, BuildSketch, Locations,
    Plane, Mode, Axis, Rectangle, RectangleRounded, Circle,
    extrude, fillet,
)


# ===== 屏幕 & PCB =====
SCREEN_W   = 44.0   # 显示区宽 (X)
SCREEN_H   = 60.6   # 显示区高 (Y)
SCREEN_T   = 2.5    # 显示区厚

PCB_W      = 45.0   # 屏 PCB 宽 (X)
PCB_H      = 73.5   # 屏 PCB 长 (Y)
PCB_T      = 1.2    # 屏 PCB 厚

# === FPC 排线 / 元件让位槽 (PCB 短边一端中央的方槽) ===
# 用户实测: 22mm (X) × 5mm (Y) × 深 1.3mm
PCB_FLEX_LEN_X = 22.0           # 沿 X 方向 (中央居中)
PCB_FLEX_LEN_Y = 5.0            # 沿 Y 方向
PCB_FLEX_DEPTH = 1.3            # 比 PCB pocket 再深 1.3mm
PCB_FLEX_END   = -1             # -1 = -Y 端 (底端), +1 = +Y 端 (顶端)

# 屏幕在 PCB 上居中, 长方向上下各预留 (73.5-60.6)/2 = 6.45 mm

# ===== 按钮 (左侧面 -X) =====
BTN_HOLE_D     = 3.3       # 圆孔直径
BTN_SPACING    = 13.1      # 相邻按钮间距
BTN_COUNT      = 3         # 共 3 个

# ===== 压力传感器 (装在底盖外底面, 表背贴手腕方向) =====
# PCB 平躺贴在下底内表面 (Z+ 内腔底), 凹槽固定; 外底面圆孔让水接触传感器圆顶
PRESS_PCB_W       = 10.2   # PCB X 方向尺寸
PRESS_PCB_H       = 13.0   # PCB Y 方向尺寸
PRESS_PCB_POCKET_D = 1.8   # PCB 凹槽深 (= PCB 厚 1.6mm + 0.2 间隙)
PRESS_PCB_GAP     = 0.3    # 凹槽周向松量
PRESS_SENSOR_D    = 3.6    # 传感器圆顶孔直径
# 传感器在 PCB 右上角偏移 (距 PCB 中心)
PRESS_SENSOR_OFFSET_X = PRESS_PCB_W / 2 - 2.5   # 距中心右侧 2.5mm
PRESS_SENSOR_OFFSET_Y = PRESS_PCB_H / 2 - 2.5   # 距中心上方 2.5mm
# PCB 凹槽中心位置 (内腔底面坐标)
# 内腔 48×76.5mm, 电池 34×51mm 居中占 ±17×±25.5, 剩余 +Y 端 12.75mm
# 把传感器放电池上方居中, 避开电池凸点 (BAT_H 在下面定义但运行时 OK)
PRESS_CENTER_X = 0.0
PRESS_CENTER_Y = 33.5       # = BAT_H/2 (25.5) + PRESS_PCB_H/2 (6.5) + 1.5 间隙

# ===== 电池 =====
BAT_T = 4.8
BAT_W = 34.0
BAT_H = 51.0

# ===== TP4056 =====
TP4056_W     = 18.0
TP4056_H     = 27.2
TP4056_T     = 3.5
TP4056_PCB_T = 1.6

# ===== USB-C (在 -Y 端面, 中间高度) =====
USBC_W = 9.0    # 标准 Type-C 母座
USBC_H = 3.3

# ===== 外壳总尺寸 =====
WALL       = 2.5
CORNER_R   = 4.0
# PCB 周边 1.5 mm 间隙 + 壁厚
CASE_W     = PCB_W + 2 * (WALL + 1.5)             # ≈ 53
CASE_H     = PCB_H + 2 * (WALL + 1.5)             # ≈ 81.5
CASE_Z     = 44.4                                  # 用户指定厚度
TOP_THICK  = 3.5                                   # 顶盖厚 (含屏幕沉槽)
BOT_THICK  = CASE_Z - TOP_THICK                    # 下底厚

# O 圈密封槽 (上下盖之间)
ORING_W      = 2.0
ORING_DEPTH  = 1.2
ORING_INSET  = 1.5

# 螺丝
SCREW_THRU_D     = 2.4
SCREW_HEAD_D     = 4.2
SCREW_HEAD_DEPTH = 2.0
SCREW_INSET      = 5.0
SCREW_POST_D     = 4.0   # 螺丝柱直径 (不能 ≥ 2*(SCREW_INSET-WALL)=5mm 否则与内壁切线接触)
SCREW_TAP_D      = 1.7
SCREW_TAP_DEPTH  = 6.0

# 螺丝 4 角位置
SCREW_LOCS = [
    (-(CASE_W/2 - SCREW_INSET), -(CASE_H/2 - SCREW_INSET)),
    ( (CASE_W/2 - SCREW_INSET), -(CASE_H/2 - SCREW_INSET)),
    (-(CASE_W/2 - SCREW_INSET),  (CASE_H/2 - SCREW_INSET)),
    ( (CASE_W/2 - SCREW_INSET),  (CASE_H/2 - SCREW_INSET)),
]

# 按钮 X 位置 (-X 长侧面外表)
BTN_X_INNER = -CASE_W / 2 + 0.5    # 略入壁内, 方便穿透
BTN_Y_LOCS  = [(i - (BTN_COUNT - 1) / 2) * BTN_SPACING for i in range(BTN_COUNT)]

# 压力传感器在 +Y 端右上角
PRESS_CY = CASE_H / 2 - WALL - PRESS_PCB_H / 2 - 3.0   # 距顶端 3mm 边距
PRESS_CZ_FROM_BOT = BOT_THICK - 5.0   # 离顶面 5mm 高位置 (装在主板对应高度)


# ============================================================
# 顶盖
# ============================================================
def make_top():
    with BuildPart() as top:
        # 主体
        with BuildSketch():
            RectangleRounded(CASE_W, CASE_H, CORNER_R)
        extrude(amount=TOP_THICK)

        # 屏幕窗口 (顶面镂空到 PCB 沉槽底)
        # PCB 在顶盖内部沉槽 (深 PCB_T = 1.2)
        # 显示区从顶面挖到外
        with BuildSketch(Plane.XY.offset(0)):
            Rectangle(SCREEN_W, SCREEN_H)
        extrude(amount=TOP_THICK, mode=Mode.SUBTRACT)

        # PCB 沉槽 (从底面挖, 让 PCB 嵌入顶盖)
        pcb_pocket_w = PCB_W + 0.4         # 45.4
        pcb_pocket_h = PCB_H + 0.4         # 73.9
        pcb_pocket_d = PCB_T + 0.3         # 1.5
        with BuildSketch(Plane.XY.offset(TOP_THICK - pcb_pocket_d)):
            Rectangle(pcb_pocket_w, pcb_pocket_h)
        extrude(amount=pcb_pocket_d + 0.1, mode=Mode.SUBTRACT)

        # === FPC 排线 / 元件让位槽 (22 × 5 × 1.3mm 方槽, 在 PCB 短边一端中央) ===
        flex_total_depth = pcb_pocket_d + PCB_FLEX_DEPTH    # 1.5 + 1.3 = 2.8mm
        # Y 中心: 距 pocket 边缘留 0.5mm, 凹槽中心
        flex_cy = PCB_FLEX_END * (pcb_pocket_h / 2 - PCB_FLEX_LEN_Y / 2 - 0.5)
        with BuildSketch(Plane.XY.offset(TOP_THICK - flex_total_depth)):
            with Locations((0, flex_cy)):
                Rectangle(PCB_FLEX_LEN_X, PCB_FLEX_LEN_Y)
        extrude(amount=flex_total_depth + 0.1, mode=Mode.SUBTRACT)

        # 4 角螺丝沉头孔 (顶面穿到底)
        with BuildSketch():
            with Locations(*SCREW_LOCS):
                Circle(SCREW_THRU_D / 2)
        extrude(amount=TOP_THICK, mode=Mode.SUBTRACT)
        with BuildSketch(Plane.XY.offset(TOP_THICK - SCREW_HEAD_DEPTH)):
            with Locations(*SCREW_LOCS):
                Circle(SCREW_HEAD_D / 2)
        extrude(amount=SCREW_HEAD_DEPTH + 0.1, mode=Mode.SUBTRACT)

    p = top.part
    p.label = "top_cover"
    return p


# ============================================================
# 下底 (主体)
# ============================================================
def make_bottom():
    with BuildPart() as bot:
        # 主体
        with BuildSketch():
            RectangleRounded(CASE_W, CASE_H, CORNER_R)
        extrude(amount=BOT_THICK)

        # 内腔 (从顶面挖, 留 BOT_THICK_WALL 厚底)
        inner_w = CASE_W - 2 * WALL
        inner_h = CASE_H - 2 * WALL
        inner_d = BOT_THICK - WALL    # 留底 WALL 厚
        with BuildSketch(Plane.XY.offset(WALL)):
            RectangleRounded(inner_w, inner_h, max(CORNER_R - WALL, 1.5))
        extrude(amount=BOT_THICK, mode=Mode.SUBTRACT)

        # 顶面 O 圈密封槽 (向下挖一圈)
        outer_w = CASE_W - 2 * ORING_INSET
        outer_h = CASE_H - 2 * ORING_INSET
        inner_or_w = outer_w - 2 * ORING_W
        inner_or_h = outer_h - 2 * ORING_W
        with BuildSketch(Plane.XY.offset(BOT_THICK - ORING_DEPTH)):
            RectangleRounded(outer_w, outer_h, max(CORNER_R - ORING_INSET, 0.5))
            RectangleRounded(inner_or_w, inner_or_h,
                              max(CORNER_R - ORING_INSET - ORING_W, 0.5),
                              mode=Mode.SUBTRACT)
        extrude(amount=ORING_DEPTH + 0.1, mode=Mode.SUBTRACT)

        # 3 按钮孔 (穿透 -X 侧壁, 起始在壁外侧 0.5mm)
        btn_z = BOT_THICK - 8.0  # 按钮中心高度
        with BuildSketch(Plane.YZ.offset(-(CASE_W / 2 + 0.5))):
            for ly in BTN_Y_LOCS:
                with Locations((ly, btn_z)):
                    Circle(BTN_HOLE_D / 2)
        # 仅 extrude 到刚穿透壁 (WALL + 0.5 起 + 0.6 = 内腔内 0.1mm)
        extrude(amount=WALL + 0.6, mode=Mode.SUBTRACT)

        # === 压力传感器 PCB 凹槽 (在内腔底面, 平躺) ===
        with BuildSketch(Plane.XY.offset(WALL)):
            with Locations((PRESS_CENTER_X, PRESS_CENTER_Y)):
                Rectangle(PRESS_PCB_W + PRESS_PCB_GAP, PRESS_PCB_H + PRESS_PCB_GAP)
        extrude(amount=-PRESS_PCB_POCKET_D, mode=Mode.SUBTRACT)

        # === 压力传感器圆孔 (外底面穿透到凹槽, 对准 PCB 右上角) ===
        sensor_x = PRESS_CENTER_X + PRESS_SENSOR_OFFSET_X
        sensor_y = PRESS_CENTER_Y + PRESS_SENSOR_OFFSET_Y
        with BuildSketch(Plane.XY.offset(-0.1)):
            with Locations((sensor_x, sensor_y)):
                Circle(PRESS_SENSOR_D / 2)
        extrude(amount=WALL + 0.2, mode=Mode.SUBTRACT)

        # USB-C 矩形孔 (-Y 端壁, 中间高度)
        usbc_z = BOT_THICK / 2
        with BuildSketch(Plane.XZ.offset(-(CASE_H / 2 + 0.5))):
            with Locations((0, usbc_z)):
                Rectangle(USBC_W, USBC_H)
        extrude(amount=WALL + 0.6, mode=Mode.SUBTRACT)

        # 4 角螺丝攻丝柱 + 攻丝孔
        with BuildSketch(Plane.XY.offset(WALL)):
            with Locations(*SCREW_LOCS):
                Circle(SCREW_POST_D / 2)
        extrude(amount=BOT_THICK - WALL - 1)
        # 攻丝孔 (从顶面向下)
        with BuildSketch(Plane.XY.offset(BOT_THICK - SCREW_TAP_DEPTH)):
            with Locations(*SCREW_LOCS):
                Circle(SCREW_TAP_D / 2)
        extrude(amount=SCREW_TAP_DEPTH + 0.1, mode=Mode.SUBTRACT)

        # 电池仓凸点 (4 角小柱, 限制电池位置)
        bat_pin_locs = [
            (-BAT_W/2 + 2, -BAT_H/2 + 2),
            ( BAT_W/2 - 2, -BAT_H/2 + 2),
            (-BAT_W/2 + 2,  BAT_H/2 - 2),
            ( BAT_W/2 - 2,  BAT_H/2 - 2),
        ]
        with BuildSketch(Plane.XY.offset(WALL)):
            with Locations(*bat_pin_locs):
                Circle(1.0)
        extrude(amount=1.0)

    p = bot.part
    p.label = "bottom_case"
    return p


if __name__ == "__main__":
    top = make_top()
    bot = make_bottom()
    print("=== TOP ===")
    print(f"Volume: {top.volume:.0f} mm³")
    print(f"BBox: {top.bounding_box().size}")
    print("=== BOTTOM ===")
    print(f"Volume: {bot.volume:.0f} mm³")
    print(f"BBox: {bot.bounding_box().size}")
