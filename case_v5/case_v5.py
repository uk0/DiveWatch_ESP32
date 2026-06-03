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
SCREEN_LIP_W = 1.5  # 屏幕窗顶部一圈 lip 宽 (单边内推, 卡住屏幕不上掉)
SCREEN_LIP_H = 1.0  # lip 厚度 (从顶面往下)
SCREEN_T   = 2.5    # 显示区厚

PCB_W      = 45.0   # 屏 PCB 宽 (X)
PCB_H      = 73.5   # 屏 PCB 长 (Y)
PCB_T      = 1.2    # 屏 PCB 厚

# === FPC 排线 / 元件让位槽 (PCB 短边一端中央的方槽) ===
# 用户实测: 22mm (X) × 5mm (Y) × 深 1.3mm
PCB_FLEX_LEN_X = 22.0           # 沿 X 方向 (中央居中)
PCB_FLEX_LEN_Y = 5.0            # 沿 Y 方向
PCB_FLEX_DEPTH = 2.5            # 比 PCB pocket 再深 2.5mm (用户要求加深)
PCB_FLEX_END   = -1             # -1 = -Y 端 (底端), +1 = +Y 端 (顶端)

# 屏幕在 PCB 上居中, 长方向上下各预留 (73.5-60.6)/2 = 6.45 mm

# ===== 按钮 (左侧面 -X) - 按钮本体 6.2x6.2mm 方块嵌入 =====
# 中间圆孔 (按钮触发) ⌀4.0, 两侧按钮孔 ⌀5.0 (加大)
# 按钮本体方坑 6.6x6.6 (= 6.2 + 0.4 装配间隙) 嵌入壁内 3mm 深, 留 0.5mm 壁让水不进
BTN_BODY_SIZE   = 6.6      # 按钮本体方坑边长 (6.2 + 0.4 间隙)
BTN_BODY_DEPTH  = 3.0      # 方坑挖入壁内深度
BTN_HOLE_CENTER = 4.0      # 中间按钮触发圆孔直径
BTN_HOLE_SIDE   = 5.0      # 两侧按钮触发圆孔直径 (用户要求加大)
BTN_SPACING     = 12.6     # 相邻按钮间距 (13.1→12.6, 让最右按钮更靠中)
BTN_COUNT       = 3        # 共 3 个

# ===== 压力传感器 (装在 +X 长侧面居中) =====
# PCB 嵌在 +X 内壁凹槽里, 凹槽间隙加大避免装不进
# 外壁圆孔 ⌀3.6 对准 PCB 右上角 (距 PCB 边缘留 3mm 边距, 不靠角)
PRESS_PCB_W       = 10.2   # PCB Z 方向尺寸 (沿外壳厚度)
PRESS_PCB_H       = 13.0   # PCB Y 方向尺寸 (沿外壳长边)
PRESS_PCB_POCKET_D = 1.8   # 凹槽深 (向 -X 内壁挖)
PRESS_PCB_GAP     = 1.0    # 凹槽周向松量 (单边 0.5mm, 加大避免装不进)
PRESS_SENSOR_D    = 3.6    # 传感器圆顶孔直径
PRESS_SENSOR_EDGE_MARGIN = 3.0   # 圆孔距 PCB 边缘距离 (距角 3mm, 不靠边)
# PCB 凹槽中心: +X 长侧面居中 (Y=0, Z=BOT_THICK/2)
PRESS_CENTER_Y = 0.0
# Z 中心在 BOT_THICK 的 1/2 位置 (即外壳厚度中部)

# ===== 电池 =====
BAT_T = 4.8
BAT_W = 34.0
BAT_H = 51.0

# ===== TP4056 =====
TP4056_W     = 18.0
TP4056_H     = 27.2
TP4056_T     = 3.5
TP4056_PCB_T = 1.6

# ===== 无线充电线圈凹槽 (装在 bottom 内腔底面, 不外露) =====
# 凹槽从内腔底面 (z=WALL) 向下挖入, 留外壁 1.2mm 让 Qi 磁场穿透
WIRELESS_COIL_OD     = 45.0           # 线圈外径 (Qi 标准 40-44mm)
WIRELESS_COIL_DEPTH  = 1.3            # 凹槽深 (向 -Z 挖, 外壁剩 WALL-DEPTH = 1.2mm)

# ===== 外壳总尺寸 =====
WALL       = 2.5
CORNER_R   = 4.0
# 外壳必须足够大让 U 槽内缘 > PCB pocket, 否则上盖会被切断
# 需求: CASE >= PCB_pocket + 2*SEAL_GROOVE_W + 2*SEAL_INSET + 2*0.5(余量) = PCB+0.4 + 6 + 4 + 1 = PCB+11.4
# PCB 周边: WALL(2.5) + SEAL_INSET(2) + SEAL_GROOVE_W(3) + 余量(0.5) = 8mm
CASE_W     = PCB_W + 2 * (WALL + 4.0)             # 45 + 13 = 58
CASE_H     = PCB_H + 2 * (WALL + 4.0)             # 73.5 + 13 = 86.5
CASE_Z     = 28.0                                  # 总厚 (参考版 26 + 2 装配)
TOP_THICK  = 7.0                                   # 顶盖 (参考版 5 + 2 装配)
BOT_THICK  = CASE_Z - TOP_THICK                    # 下底厚 = 21.0 (不变)

# ============================================================
# 防水密封: 上下盖凹凸契合结构 + O 圈
# ============================================================
# 下底顶面: 梯形凸起 lip (底宽 4mm 与主体融合, 顶宽 2mm 卡入 top 凹槽)
# 上盖顶面: U 形凹槽 (装 ⌀2mm O 圈)
# 梯形 lip 底部宽 → 与 bottom 主体接触面积大, 不易掉; 顶部窄 → 卡入 top 凹槽
SEAL_INSET     = 2.0   # 密封线距外壁内退距离
SEAL_GROOVE_W  = 3.0   # U 形凹槽宽度 (top)
SEAL_GROOVE_D  = 2.0   # U 形凹槽深度
SEAL_LIP_TOP_W = 2.0   # 凸起顶部宽度 (卡入凹槽部分)
SEAL_LIP_BOT_W = 4.0   # 凸起底部宽度 (与 bottom 主体融合, 加大接触面防掉)
SEAL_LIP_H     = 1.5   # 凸起高度
SEAL_LIP_GAP   = 0.5   # 凸起顶部两侧装配松量

# ===== 表带耳 (两端各 2 个) =====
STRAP_WIDTH    = 24.0   # 表带宽 = 两耳间距
LUG_THICK      = 4.0    # 单个耳片 X 厚度
LUG_OUT        = 6.0    # 耳片向 ±Y 凸出长度
LUG_HEIGHT     = 8.0    # 耳片 Z 高度
LUG_Z_CENTER   = 10.5   # 耳片 Z 中心 (BOT_THICK 21 的中点)
SPRING_BAR_D   = 2.5    # 弹簧棒孔直径
LUG_FILLET_R   = 1.5    # 耳片末端圆角

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
    """
    顶盖结构 (z=0 显示面, z=TOP_THICK 装配面):
    屏幕相关深度与参考版 5mm 完全一致 (距显示面不变),
    多出的 2mm 全部在装配面一侧 (PCB 沉槽底 → 装配面之间).

      z=0   ───── 显示面 (屏幕玻璃露出)
      │ 屏幕窗 44×60.6 (贯穿)
      │
      z=3   ───── FPC 排线槽底 (距显示面 3mm, 同参考版)
      z=3.5 ───── PCB 台阶 (距显示面 3.5mm, 同参考版)
      │ PCB 沉槽 45.4×73.9 (1.5mm 深)
      │ ← 4 个凸起在此处卡 PCB
      z=5   ───── 参考版装配面 (原来到这就结束)
      │ 新增 2mm 空间 (装配余量)
      z=5   ───── 密封槽 (2mm 深)
      z=7   ───── 装配面 (对接 bottom)
    """
    with BuildPart() as top:
        # 主体
        with BuildSketch():
            RectangleRounded(CASE_W, CASE_H, CORNER_R)
        extrude(amount=TOP_THICK)

        # 1. 屏幕窗 (贯穿 z=0..TOP_THICK)
        with BuildSketch(Plane.XY.offset(0)):
            Rectangle(SCREEN_W, SCREEN_H)
        extrude(amount=TOP_THICK, mode=Mode.SUBTRACT)

        # 2. PCB 沉槽 (深度/位置与参考版一致: 距显示面 3.5mm 处开始, 1.5mm 深)
        #    PCB 比屏幕大 → 形成天然台阶; 凸起额外加固
        pcb_pocket_w = PCB_W + 0.4         # 45.4
        pcb_pocket_h = PCB_H + 0.4         # 73.9
        pcb_pocket_d = PCB_T + 0.3         # 1.5
        # 距显示面 3.5mm = 与参考版完全相同 (参考版 5 - 1.5 = 3.5)
        pcb_ledge_z  = TOP_THICK - pcb_pocket_d - 2.0   # 7 - 1.5 - 2 = 3.5
        with BuildSketch(Plane.XY.offset(pcb_ledge_z)):
            Rectangle(pcb_pocket_w, pcb_pocket_h)
        extrude(amount=pcb_pocket_d + 2.0 + 0.1, mode=Mode.SUBTRACT)  # 挖穿到装配面

        # 3. FPC 排线槽 (距显示面同参考版)
        flex_total_depth = pcb_pocket_d + PCB_FLEX_DEPTH   # 1.5 + 2.5 = 4.0
        flex_top_z = TOP_THICK - flex_total_depth - 2.0    # 7 - 4 - 2 = 1.0
        flex_cy = PCB_FLEX_END * (pcb_pocket_h / 2 - PCB_FLEX_LEN_Y / 2 - 0.5)
        with BuildSketch(Plane.XY.offset(flex_top_z)):
            with Locations((0, flex_cy)):
                Rectangle(PCB_FLEX_LEN_X, PCB_FLEX_LEN_Y)
        extrude(amount=TOP_THICK - flex_top_z + 0.1, mode=Mode.SUBTRACT)

        # 4. 密封 U 形凹槽 (装配面挖, 对接 bottom)
        groove_out_w = CASE_W - 2 * SEAL_INSET
        groove_out_h = CASE_H - 2 * SEAL_INSET
        groove_in_w  = groove_out_w - 2 * SEAL_GROOVE_W
        groove_in_h  = groove_out_h - 2 * SEAL_GROOVE_W
        with BuildSketch(Plane.XY.offset(TOP_THICK - SEAL_GROOVE_D)):
            RectangleRounded(groove_out_w, groove_out_h,
                              max(CORNER_R - SEAL_INSET, 0.5))
            RectangleRounded(groove_in_w, groove_in_h,
                              max(CORNER_R - SEAL_INSET - SEAL_GROOVE_W, 0.5),
                              mode=Mode.SUBTRACT)
        extrude(amount=SEAL_GROOVE_D + 0.1, mode=Mode.SUBTRACT)

        # 5. 4 角螺丝沉头孔
        with BuildSketch():
            with Locations(*SCREW_LOCS):
                Circle(SCREW_THRU_D / 2)
        extrude(amount=TOP_THICK, mode=Mode.SUBTRACT)
        with BuildSketch(Plane.XY.offset(TOP_THICK - SCREW_HEAD_DEPTH)):
            with Locations(*SCREW_LOCS):
                Circle(SCREW_HEAD_D / 2)
        extrude(amount=SCREW_HEAD_DEPTH + 0.1, mode=Mode.SUBTRACT)

        # 6. 4 个凸起在 PCB 台阶处卡住 PCB (不是卡屏幕)
        #    位于 PCB 沉槽壁内, 从壁面向内突出, PCB 装入后被卡住
        CLIP_W   = 4.0    # 沿壁面方向宽度
        CLIP_D   = 0.5    # 向内突出深度 (沉槽单边间隙 0.2, 凸起 0.5 形成过盈卡扣)
        CLIP_H   = 1.0    # Z 方向高度
        clip_z   = pcb_ledge_z + 0.8   # 台阶 + PCB 厚 0.6 + 0.2 间隙, 凸起压住 PCB 顶面
        # ±X 两边 (PCB 沉槽短边中点)
        for sx in (-1, 1):
            cx = sx * (pcb_pocket_w / 2 - CLIP_D / 2)
            with BuildSketch(Plane.XY.offset(clip_z)):
                with Locations((cx, 0)):
                    Rectangle(CLIP_D, CLIP_W)
            extrude(amount=CLIP_H, mode=Mode.ADD)
        # ±Y 两边 (PCB 沉槽长边中点)
        for sy in (-1, 1):
            cy = sy * (pcb_pocket_h / 2 - CLIP_D / 2)
            with BuildSketch(Plane.XY.offset(clip_z)):
                with Locations((0, cy)):
                    Rectangle(CLIP_W, CLIP_D)
            extrude(amount=CLIP_H, mode=Mode.ADD)

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

        # === 内腔顶部 斜坡 ledge 支撑 lip 底部 (4 层模拟斜坡 防 3.5mm 悬空) ===
        # 总高 4mm, 从 z=BOT_THICK-4 处 ledge 宽 1mm 逐层向内推到 z=BOT_THICK 处 3.5mm
        # 顶层 3.5mm 完全支撑 lip, 下层逐渐缩窄斜坡延伸到外壁顶面下方 4mm
        # 斜面打印每层悬空 ≤0.8mm 远小于 FDM 极限
        LEDGE_W_TOP = SEAL_LIP_BOT_W - (WALL - SEAL_INSET)   # = 3.5 顶层宽
        LEDGE_H_TOTAL = 4.0
        LAYERS = 4
        for i in range(LAYERS):
            t = (i + 1) / LAYERS                              # 0.25, 0.5, 0.75, 1.0
            lw = LEDGE_W_TOP * t                              # 0.875, 1.75, 2.625, 3.5
            z_lo = BOT_THICK - LEDGE_H_TOTAL + i * (LEDGE_H_TOTAL / LAYERS)
            z_hi = z_lo + (LEDGE_H_TOTAL / LAYERS)
            ledge_in_w = inner_w - 2 * lw
            ledge_in_h = inner_h - 2 * lw
            with BuildSketch(Plane.XY.offset(z_lo)):
                RectangleRounded(inner_w, inner_h, max(CORNER_R - WALL, 1.5))
                RectangleRounded(ledge_in_w, ledge_in_h,
                                  max(CORNER_R - WALL - lw, 0.5),
                                  mode=Mode.SUBTRACT)
            extrude(amount=z_hi - z_lo, mode=Mode.ADD)

        # === 梯形密封 lip (底宽 4mm 与主体融合, 顶宽 2mm 卡入 top 凹槽) ===
        # 由两段 extrude 模拟梯形: 底层 (高 0.5mm, 宽 BOT 4mm) + 顶层 (高 1.0mm, 宽 TOP 2mm)
        # 这样接触面 = 4mm 宽周长 → 远大于 2mm 凸起, 不易掉
        for layer in [0, 1]:
            if layer == 0:
                lw   = SEAL_LIP_BOT_W
                z_lo = BOT_THICK
                z_hi = BOT_THICK + 0.5
            else:
                lw   = SEAL_LIP_TOP_W - SEAL_LIP_GAP   # 顶部留装配松量
                z_lo = BOT_THICK + 0.5
                z_hi = BOT_THICK + SEAL_LIP_H
            lip_out_w = CASE_W - 2 * SEAL_INSET
            lip_out_h = CASE_H - 2 * SEAL_INSET
            lip_in_w  = lip_out_w - 2 * lw
            lip_in_h  = lip_out_h - 2 * lw
            with BuildSketch(Plane.XY.offset(z_lo)):
                RectangleRounded(lip_out_w, lip_out_h,
                                  max(CORNER_R - SEAL_INSET, 0.5))
                RectangleRounded(lip_in_w, lip_in_h,
                                  max(CORNER_R - SEAL_INSET - lw, 0.5),
                                  mode=Mode.SUBTRACT)
            extrude(amount=z_hi - z_lo, mode=Mode.ADD)

        # === 3 按钮孔 (穿透 -X 侧壁) Z 中心居中于 bottom 厚度 ===
        btn_z = BOT_THICK / 2.0   # 上下居中 = 21/2 = 10.5
        # 按钮本体方坑 (从外壁向内挖 BTN_BODY_DEPTH mm, 嵌入 6.2x6.2 按钮)
        with BuildSketch(Plane.YZ.offset(-(CASE_W / 2 + 0.1))):
            for ly in BTN_Y_LOCS:
                with Locations((ly, btn_z)):
                    Rectangle(BTN_BODY_SIZE, BTN_BODY_SIZE)
        extrude(amount=BTN_BODY_DEPTH, mode=Mode.SUBTRACT)
        # 中间按钮触发圆孔 ⌀4.0 (穿透壁内残留部分)
        with BuildSketch(Plane.YZ.offset(-(CASE_W / 2 + 0.1))):
            with Locations((BTN_Y_LOCS[1], btn_z)):
                Circle(BTN_HOLE_CENTER / 2)
        extrude(amount=WALL + 0.5, mode=Mode.SUBTRACT)
        # 两侧按钮触发圆孔 ⌀5.0 (用户要求加大)
        with BuildSketch(Plane.YZ.offset(-(CASE_W / 2 + 0.1))):
            with Locations((BTN_Y_LOCS[0], btn_z), (BTN_Y_LOCS[2], btn_z)):
                Circle(BTN_HOLE_SIDE / 2)
        extrude(amount=WALL + 0.5, mode=Mode.SUBTRACT)

        # === 压力传感器 PCB 凹槽 (从 +X 内壁挖, 中心 Z 同按钮高度) ===
        press_cz = btn_z   # 与按钮同高, 整体向下移
        with BuildSketch(Plane.YZ.offset(CASE_W / 2 - WALL)):
            with Locations((PRESS_CENTER_Y, press_cz)):
                Rectangle(PRESS_PCB_H + PRESS_PCB_GAP, PRESS_PCB_W + PRESS_PCB_GAP)
        extrude(amount=-PRESS_PCB_POCKET_D, mode=Mode.SUBTRACT)

        # === 压力传感器圆孔 (+X 外壁穿入, PCB 右上角偏移) ===
        sensor_cy = PRESS_CENTER_Y + (PRESS_PCB_H / 2 - PRESS_SENSOR_EDGE_MARGIN)
        sensor_cz = press_cz + (PRESS_PCB_W / 2 - PRESS_SENSOR_EDGE_MARGIN)
        with BuildSketch(Plane.YZ.offset(CASE_W / 2 + 0.1)):
            with Locations((sensor_cy, sensor_cz)):
                Circle(PRESS_SENSOR_D / 2)
        extrude(amount=-(WALL + 0.2), mode=Mode.SUBTRACT)

        # === 无线充电线圈凹槽 (内腔底面, 不在外底面) ===
        # 从内腔底面 z=WALL 向 +Z 挖 WIRELESS_COIL_DEPTH 凹槽固定线圈
        # 实际:让线圈卡在 z=[WALL-DEPTH=1.2, WALL=2.5] 区间, 外底面 z=0 平整
        # 即从外底面 z=0 向上挖到 z=DEPTH, 但留四周 (内腔大于线圈) 的部分作为线圈卡位
        # 简化: 从内腔底面 z=WALL 向下 (-Z) 挖凹槽, 但凹槽底距外底 = WALL - DEPTH
        with BuildSketch(Plane.XY.offset(WALL - WIRELESS_COIL_DEPTH)):
            Circle(WIRELESS_COIL_OD / 2)
        extrude(amount=WIRELESS_COIL_DEPTH + 0.1, mode=Mode.SUBTRACT)

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

        # === 表带耳 (±Y 端面各 2 个, 间距 STRAP_WIDTH) ===
        # 每端 2 个耳片对称分布, 中心 X = ±STRAP_WIDTH/2
        # 耳片整体凸出 ±Y 方向, 形成手表表带连接点
        lug_x_offset = STRAP_WIDTH / 2
        for sx in (-1, 1):       # X 方向左右两个耳片
            for sy in (-1, 1):   # Y 方向 -Y / +Y 两端
                cx = sx * lug_x_offset
                cy = sy * (CASE_H / 2 + LUG_OUT / 2)
                # 耳片本体
                with BuildSketch(Plane.XY.offset(LUG_Z_CENTER - LUG_HEIGHT/2)):
                    with Locations((cx, cy)):
                        RectangleRounded(LUG_THICK, LUG_OUT, LUG_FILLET_R)
                extrude(amount=LUG_HEIGHT)

        # === 弹簧棒孔 (沿 X 方向穿透 2 个对应耳片) ===
        # 每端一根棒, 中心 Y 在耳片中央
        for sy in (-1, 1):
            cy = sy * (CASE_H / 2 + LUG_OUT / 2)
            # 从 -X 方向穿入, 穿透整个 STRAP_WIDTH + 2*LUG_THICK
            with BuildSketch(Plane.YZ.offset(-(STRAP_WIDTH/2 + LUG_THICK + 1))):
                with Locations((cy, LUG_Z_CENTER)):
                    Circle(SPRING_BAR_D / 2)
            extrude(amount=STRAP_WIDTH + 2*LUG_THICK + 2, mode=Mode.SUBTRACT)

    p = bot.part
    p.label = "bottom_case"
    return p


def make_tpu_bumper():
    """
    TPU 软质保护套, 包裹 top+bottom 完整组件 (CASE_W × CASE_H × CASE_Z)
    - 单边 TPU_OUT=2.5mm 外推
    - 内腔比外壳大 TPU_GAP=0.3mm 单边, 利用 TPU 弹性套入
    - 顶面开屏幕窗 (露 SCREEN + 2mm 边距)
    - 底面开无线充电窗 (圆形, 直径 ≥ WIRELESS_COIL_OD)
    - -X 侧 3 个按钮圆孔, +X 侧 1 个传感器圆孔
    - ±Y 端完全开口让表带耳 + 弹簧棒 + 表带穿出 (Apple Watch 风格)
    - 整个外形圆角加大, 视觉柔和
    """
    TPU_OUT  = 2.5     # 单边 TPU 厚
    TPU_GAP  = 0.3     # 内腔单边间隙, 利用弹性套入
    BTN_HOLE = 5.5     # 按钮孔直径 (BTN_HOLE_SIDE + 1)
    SENS_HOLE = 4.6    # 传感器孔直径 (PRESS_SENSOR_D + 1)
    SCR_WIN_M = 2.0    # 屏幕窗四周边距 (相对 SCREEN)
    CHG_WIN_R = WIRELESS_COIL_OD / 2 + 1   # 充电窗半径
    STRAP_OPEN = STRAP_WIDTH + 4           # ±Y 端开口宽度 (露耳片+表带)

    out_w  = CASE_W + 2 * TPU_OUT
    out_h  = CASE_H + 2 * TPU_OUT
    out_z  = CASE_Z + 2 * TPU_OUT
    in_w   = CASE_W + 2 * TPU_GAP
    in_h   = CASE_H + 2 * TPU_GAP
    in_z   = CASE_Z + 2 * TPU_GAP
    tpu_corner = CORNER_R + TPU_OUT + 1.0

    with BuildPart() as bumper:
        # 外形 (大圆角)
        with BuildSketch():
            RectangleRounded(out_w, out_h, tpu_corner)
        extrude(amount=out_z)
        # 减内腔 (与 case 同形状 + 0.3mm 单边间隙)
        # case 实际占 z = 0..CASE_Z, 在 bumper 中居中: z = TPU_OUT..TPU_OUT+CASE_Z
        # 内腔做 z = TPU_OUT - TPU_GAP .. TPU_OUT + CASE_Z + TPU_GAP
        with BuildSketch(Plane.XY.offset(TPU_OUT - TPU_GAP)):
            RectangleRounded(in_w, in_h, CORNER_R + TPU_GAP)
        extrude(amount=in_z, mode=Mode.SUBTRACT)

        # 屏幕窗 (顶面 -Z 削穿 TPU 顶皮)
        scr_w = SCREEN_W + 2 * SCR_WIN_M
        scr_h = SCREEN_H + 2 * SCR_WIN_M
        with BuildSketch(Plane.XY.offset(out_z + 0.05)):
            RectangleRounded(scr_w, scr_h, 3.5)
        extrude(amount=-(TPU_OUT + 1.0), mode=Mode.SUBTRACT)

        # 无线充电窗 (底面圆孔)
        with BuildSketch(Plane.XY.offset(-0.05)):
            Circle(CHG_WIN_R)
        extrude(amount=TPU_OUT + 1.0, mode=Mode.SUBTRACT)

        # ±Y 端开口 (表带耳 + 表带穿出) — 两端对称
        # bumper 实体 z = 0..out_z, 耳片中心在 z = TPU_OUT + LUG_Z_CENTER
        # 开口需对准耳片高度, 高度覆盖 LUG_HEIGHT + 余量, 宽度覆盖表带 STRAP_OPEN
        # 用 XY 平面 sketch 在端部 Y 位置拉伸出 Y 方向的通孔, 避免 XZ 平面法线歧义
        lug_z_tpu  = TPU_OUT + LUG_Z_CENTER          # 耳片中心 Z (= 2.5 + 10.5 = 13)
        open_z_lo  = lug_z_tpu - LUG_HEIGHT / 2 - 2   # 开口底
        open_z_hi  = lug_z_tpu + LUG_HEIGHT / 2 + 2   # 开口顶
        # 端部 Y 起点: 内腔边缘 (in_h/2) 向外挖穿整个端壁 (到 out_h/2 外)
        y_inner = in_h / 2 - 1.0      # 从内腔壁内侧一点开始
        y_outer = out_h / 2 + 1.0     # 挖到外表面外
        y_len   = y_outer - y_inner
        for sy in (-1, 1):
            with BuildSketch(Plane.XY.offset(open_z_lo)):
                with Locations((0, sy * (y_inner + y_len / 2))):
                    Rectangle(STRAP_OPEN, y_len)
            extrude(amount=open_z_hi - open_z_lo, mode=Mode.SUBTRACT)

        # -X 侧 3 按钮圆孔
        # case 中 btn_z 是相对 case Z 原点; bumper 中 case 位于 z=TPU_OUT..TPU_OUT+CASE_Z
        btn_z_tpu = TPU_OUT + (BOT_THICK / 2.0)   # 与 make_bottom 中 btn_z 一致
        for ly in BTN_Y_LOCS:
            with BuildSketch(Plane.YZ.offset(-(out_w / 2 + 0.1))):
                with Locations((ly, btn_z_tpu)):
                    Circle(BTN_HOLE / 2)
            extrude(amount=out_w + 0.2, mode=Mode.SUBTRACT)

        # +X 侧 传感器孔 (与 make_bottom sensor 位置一致)
        press_cz_tpu = TPU_OUT + (BOT_THICK / 2.0)
        sensor_cy_tpu = PRESS_CENTER_Y + (PRESS_PCB_H / 2 - PRESS_SENSOR_EDGE_MARGIN)
        sensor_cz_tpu = press_cz_tpu + (PRESS_PCB_W / 2 - PRESS_SENSOR_EDGE_MARGIN)
        with BuildSketch(Plane.YZ.offset(out_w / 2 + 0.1)):
            with Locations((sensor_cy_tpu, sensor_cz_tpu)):
                Circle(SENS_HOLE / 2)
        extrude(amount=-(out_w + 0.2), mode=Mode.SUBTRACT)

    p = bumper.part
    p.label = "tpu_bumper"
    return p


if __name__ == "__main__":
    import os, subprocess
    from build123d import export_stl, export_step
    here = os.path.dirname(os.path.abspath(__file__))
    case_dir = os.path.normpath(os.path.join(here, "..", "case"))
    os.makedirs(case_dir, exist_ok=True)

    top = make_top()
    bot = make_bottom()
    tpu = make_tpu_bumper()
    print("=== TOP ===")
    print(f"Volume: {top.volume:.0f} mm³, BBox: {top.bounding_box().size}")
    print("=== BOTTOM ===")
    print(f"Volume: {bot.volume:.0f} mm³, BBox: {bot.bounding_box().size}")
    print("=== TPU BUMPER ===")
    print(f"Volume: {tpu.volume:.0f} mm³, BBox: {tpu.bounding_box().size}")

    outs = [
        ("top_cover",   top),
        ("bottom_case", bot),
        ("tpu_bumper",  tpu),
    ]
    skill_stl_to_3mf = os.path.expanduser("~/.claude/skills/bambu-3mf/scripts/stl_to_3mf.py")
    for name, part in outs:
        step_p = os.path.join(case_dir, f"{name}.step")
        stl_p  = os.path.join(case_dir, f"{name}.stl")
        export_step(part, step_p)
        export_stl(part, stl_p)
        print(f"导出 {step_p}, {stl_p}")
        if os.path.exists(skill_stl_to_3mf):
            tmf = os.path.join(case_dir, f"{name}.3mf")
            subprocess.run(["python3", skill_stl_to_3mf, stl_p, tmf], check=True)
            print(f"3MF: {tmf}")
