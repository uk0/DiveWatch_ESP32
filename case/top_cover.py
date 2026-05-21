"""DiveWatch v4 case - top cover.

Hardware fit:
- ESP32-S3-Nano       45 x 18 x 4 mm
- ST7789 2.4" SPI     50 x 67 x 4 mm  (display 37 x 49 mm)
- 3 momentary buttons (left side, 14 mm spacing)
- M2 x 6 mm screws    (4 corners, countersunk)
- 22 mm watch strap   (top & bottom lugs)
- O-ring 2 mm cross-section (perimeter ~290 mm)

Coordinate system:
- Origin: geometric center of cover, XY = base plane
- +Z = outward (top face), -Z = mating face to bottom case
"""

from build123d import (
    BuildPart, BuildSketch, Locations,
    Plane, Mode, Axis, Rectangle, RectangleRounded, Circle,
    extrude, fillet,
)


# ---------- Parameters (mm) ----------
CASE_W      = 70.0
CASE_H      = 85.0
TOP_THICK   = 12.0
WALL        = 2.5
CORNER_R    = 5.0

CAVITY_W    = CASE_W - 2 * WALL
CAVITY_H    = CASE_H - 2 * WALL

SCREEN_W    = 38.0
SCREEN_H    = 50.0
SCREEN_OFFY = 6.0
LENS_REBATE_DEPTH = 1.0
LENS_W      = SCREEN_W + 4
LENS_H      = SCREEN_H + 4

BTN_HOLE_D    = 7.0
BTN_SPACING_Y = 14.0
BTN_OFFSET_X  = -(CASE_W/2 - 7)
BTN_CENTER_Y  = -8.0

BUZZ_HOLE_D = 1.5
BUZZ_POS    = (CASE_W/2 - 8, CASE_H/2 - 8)

SCREW_THRU_D     = 2.4
SCREW_HEAD_D     = 4.2
SCREW_HEAD_DEPTH = 2.0
SCREW_INSET      = 5.0

ORING_GROOVE_W     = 2.0
ORING_GROOVE_DEPTH = 1.5
ORING_INSET        = 1.5

STRAP_W      = 22.0
LUG_OUTREACH = 4.0
LUG_PIN_D    = 2.5


def gen_step():
    with BuildPart() as top:
        # === 主体 ===
        with BuildSketch():
            RectangleRounded(CASE_W, CASE_H, CORNER_R)
        extrude(amount=TOP_THICK)

        # === 表带耳 (Y 两端凸耳) ===
        with BuildSketch():
            with Locations((0,  (CASE_H/2 + LUG_OUTREACH/2)),
                           (0, -(CASE_H/2 + LUG_OUTREACH/2))):
                Rectangle(STRAP_W, LUG_OUTREACH + 2)
        extrude(amount=TOP_THICK)

        # === 内腔 (从底面挖) ===
        with BuildSketch(Plane.XY.offset(WALL)):
            RectangleRounded(CAVITY_W, CAVITY_H, max(CORNER_R - WALL, 1.5))
        extrude(amount=TOP_THICK, mode=Mode.SUBTRACT)

        # === 显示窗口 (从顶面挖到内腔) ===
        screen_y = -SCREEN_OFFY
        with BuildSketch(Plane.XY.offset(TOP_THICK - WALL/2)):
            with Locations((0, screen_y)):
                Rectangle(SCREEN_W, SCREEN_H)
        extrude(amount=WALL, mode=Mode.SUBTRACT)

        # === PMMA 透镜沉孔槽 (顶面更大浅槽嵌入透镜) ===
        with BuildSketch(Plane.XY.offset(TOP_THICK - LENS_REBATE_DEPTH)):
            with Locations((0, screen_y)):
                Rectangle(LENS_W, LENS_H)
        extrude(amount=LENS_REBATE_DEPTH, mode=Mode.SUBTRACT)

        # === 3 按钮孔 (左侧) ===
        btn_locs = [(BTN_OFFSET_X, BTN_CENTER_Y + (i - 1) * BTN_SPACING_Y) for i in range(3)]
        with BuildSketch():
            with Locations(*btn_locs):
                Circle(BTN_HOLE_D/2)
        extrude(amount=TOP_THICK, mode=Mode.SUBTRACT)

        # === 蜂鸣器透音孔 (右上) ===
        with BuildSketch():
            with Locations(BUZZ_POS):
                Circle(BUZZ_HOLE_D/2)
        extrude(amount=TOP_THICK, mode=Mode.SUBTRACT)

        # === 螺丝沉头孔 (4 角通孔 + 顶面沉头) ===
        corner_locs = [
            (-(CASE_W/2 - SCREW_INSET), -(CASE_H/2 - SCREW_INSET)),
            ( (CASE_W/2 - SCREW_INSET), -(CASE_H/2 - SCREW_INSET)),
            (-(CASE_W/2 - SCREW_INSET),  (CASE_H/2 - SCREW_INSET)),
            ( (CASE_W/2 - SCREW_INSET),  (CASE_H/2 - SCREW_INSET)),
        ]
        # 通孔
        with BuildSketch():
            with Locations(*corner_locs):
                Circle(SCREW_THRU_D/2)
        extrude(amount=TOP_THICK, mode=Mode.SUBTRACT)
        # 沉头 (顶面下凹)
        with BuildSketch(Plane.XY.offset(TOP_THICK - SCREW_HEAD_DEPTH)):
            with Locations(*corner_locs):
                Circle(SCREW_HEAD_D/2)
        extrude(amount=SCREW_HEAD_DEPTH + 0.1, mode=Mode.SUBTRACT)

        # === O 圈密封槽 (底面边缘内圈, 环形) ===
        outer_w = CASE_W - 2 * ORING_INSET
        outer_h = CASE_H - 2 * ORING_INSET
        with BuildSketch():
            RectangleRounded(outer_w, outer_h, max(CORNER_R - ORING_INSET, 0.5))
            RectangleRounded(outer_w - 2 * ORING_GROOVE_W,
                             outer_h - 2 * ORING_GROOVE_W,
                             max(CORNER_R - ORING_INSET - ORING_GROOVE_W, 0.5),
                             mode=Mode.SUBTRACT)
        extrude(amount=ORING_GROOVE_DEPTH, mode=Mode.SUBTRACT)

        # === 表带耳弹簧棒孔 (X 方向穿透 22mm) ===
        with BuildSketch(Plane.YZ.offset(-(STRAP_W/2 + 1))):
            with Locations(( (CASE_H/2 + LUG_OUTREACH/2),  TOP_THICK/2),
                           (-(CASE_H/2 + LUG_OUTREACH/2),  TOP_THICK/2)):
                Circle(LUG_PIN_D/2)
        extrude(amount=STRAP_W + 2, mode=Mode.SUBTRACT)

        # === 顶面外圆角 ===
        top_edges = top.edges().filter_by_position(Axis.Z,
                                                    minimum=TOP_THICK - 0.01,
                                                    maximum=TOP_THICK + 0.01)
        if top_edges:
            try:
                fillet(top_edges, 1.2)
            except Exception:
                pass

    part = top.part
    part.label = "top_cover"
    return part


if __name__ == "__main__":
    s = gen_step()
    print(f"Volume: {s.volume:.2f} mm^3")
    print(f"BBox: {s.bounding_box().size}")
