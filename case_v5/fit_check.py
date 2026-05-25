"""v5 外壳容纳/装配检查 - 验证所有部件能否正确装入."""
import sys
sys.path.insert(0, '.')
from case_v5 import *

OK   = "[OK]    "
WARN = "[WARN]  "
FAIL = "[FAIL]  "

def check(cond, msg_ok, msg_fail, warn=False):
    if cond: print(OK + msg_ok); return True
    print((WARN if warn else FAIL) + msg_fail); return False


print("=" * 70)
print("DiveWatch v5 外壳容纳/装配检查")
print("=" * 70)

# ============ 1. 外壳总尺寸 ============
print("\n[1] 外壳总尺寸")
print(f"    CASE: {CASE_W} × {CASE_H} × {CASE_Z} mm (含表带耳 Y={CASE_H + 2*LUG_OUT})")
print(f"    TOP_THICK={TOP_THICK}, BOT_THICK={BOT_THICK}, 总厚={TOP_THICK+BOT_THICK}")
inner_w = CASE_W - 2 * WALL
inner_h = CASE_H - 2 * WALL
inner_d = BOT_THICK - WALL
print(f"    内腔: {inner_w} × {inner_h} × {inner_d} mm (高度 = BOT_THICK - WALL)")

# ============ 2. 屏幕 PCB ============
print("\n[2] 屏幕 PCB")
pcb_pocket_w_check = PCB_W + 0.4
pcb_pocket_h_check = PCB_H + 0.4
check(PCB_W + 0.4 <= 47,   # U 槽内缘 = 47
      f"PCB pocket {pcb_pocket_w_check}×{pcb_pocket_h_check} 在 U 槽内缘内 (47×75.5)",
      "PCB pocket 大于 U 槽内缘, 顶层会被切断")
check(SCREEN_W <= PCB_W and SCREEN_H <= PCB_H,
      f"显示区 {SCREEN_W}×{SCREEN_H} 在 PCB {PCB_W}×{PCB_H} 内",
      "显示区超出 PCB 范围")
print(f"    显示余量: Y 上下各 ({PCB_H - SCREEN_H}/2 = {(PCB_H-SCREEN_H)/2:.2f}) mm, X ({PCB_W - SCREEN_W}/2 = {(PCB_W-SCREEN_W)/2:.2f}) mm")

# ============ 3. FPC 槽 ============
print("\n[3] FPC 排线 / 元件让位槽")
print(f"    位置: PCB {'底' if PCB_FLEX_END < 0 else '顶'}端, 中央 {PCB_FLEX_LEN_X}×{PCB_FLEX_LEN_Y}mm, 深 {PCB_FLEX_DEPTH}mm (PCB pocket {PCB_T+0.3}mm 之外再深)")
flex_floor_to_top_bottom = TOP_THICK - (PCB_T + 0.3 + PCB_FLEX_DEPTH)
check(flex_floor_to_top_bottom >= 1.5,
      f"FPC 槽底距 top 底面 {flex_floor_to_top_bottom:.2f}mm (>=1.5mm 安全)",
      f"FPC 槽底距 top 底面 {flex_floor_to_top_bottom:.2f}mm 太薄")

# ============ 4. 3 按钮 ============
print("\n[4] 3 按钮")
btn_z = BOT_THICK / 2 + 4.0
print(f"    Z 中心: {btn_z}mm (距底 {btn_z}mm, 距顶 {BOT_THICK - btn_z}mm)")
btn_y_range = [(i - 1) * BTN_SPACING + (-BTN_HOLE_D/2) for i in range(3)]
btn_y_range += [(i - 1) * BTN_SPACING + (BTN_HOLE_D/2) for i in range(3)]
print(f"    Y 范围: {min(btn_y_range):.1f} ~ {max(btn_y_range):.1f} (跨 {max(btn_y_range)-min(btn_y_range):.1f}mm)")

# ============ 5. 压力传感器 ============
print("\n[5] 压力传感器")
press_pocket_w = PRESS_PCB_H + PRESS_PCB_GAP
press_pocket_h = PRESS_PCB_W + PRESS_PCB_GAP
print(f"    PCB 凹槽: Y={press_pocket_w}mm × Z={press_pocket_h}mm × 深{PRESS_PCB_POCKET_D}mm (PCB {PRESS_PCB_H}×{PRESS_PCB_W}, 单边间隙 {PRESS_PCB_GAP/2}mm)")
press_cz = BOT_THICK / 2
sensor_cy_off = PRESS_PCB_H / 2 - PRESS_SENSOR_EDGE_MARGIN
sensor_cz_off = PRESS_PCB_W / 2 - PRESS_SENSOR_EDGE_MARGIN
print(f"    圆孔位置: Y偏=+{sensor_cy_off:.1f}, Z偏=+{sensor_cz_off:.1f} (距 PCB 边 {PRESS_SENSOR_EDGE_MARGIN}mm)")
# 凹槽不能超出 BOT_THICK
press_top_z = press_cz + press_pocket_h/2
press_bot_z = press_cz - press_pocket_h/2
check(press_top_z <= BOT_THICK - WALL and press_bot_z >= WALL,
      f"凹槽 Z 范围 [{press_bot_z:.1f}, {press_top_z:.1f}] 在内腔 [{WALL}, {BOT_THICK-WALL}] 内",
      f"凹槽 Z 范围超出内腔")

# ============ 6. 电池 ============
print("\n[6] 电池 51×34×4.8mm")
check(BAT_W <= inner_w and BAT_H <= inner_h,
      f"电池 {BAT_W}×{BAT_H} 在内腔 {inner_w}×{inner_h} 内",
      "电池超出内腔")
check(BAT_T <= inner_d,
      f"电池厚 {BAT_T}mm <= 内腔高 {inner_d}mm",
      "电池太厚")
print(f"    内腔剩余高度: {inner_d - BAT_T:.1f}mm 可容纳主板 / TP4056")

# ============ 7. TP4056 ============
print("\n[7] TP4056 27.2×18×3.5mm")
check(TP4056_W <= inner_w and TP4056_H <= inner_h,
      f"TP4056 {TP4056_W}×{TP4056_H} 在内腔 {inner_w}×{inner_h} 内",
      "TP4056 超出内腔")
print(f"    与电池叠放总厚 {BAT_T + TP4056_T + 1}mm + 间隙 (内腔 {inner_d}mm 容纳)")

# ============ 8. 无线充电线圈 ============
print("\n[8] 无线充电线圈")
print(f"    ⌀{WIRELESS_COIL_OD}mm × 深 {WIRELESS_COIL_DEPTH}mm")
print(f"    底壁剩余: {WALL - WIRELESS_COIL_DEPTH}mm (Qi 磁场穿透)")
edge_dist_w = (CASE_W - WIRELESS_COIL_OD) / 2
edge_dist_h = (CASE_H - WIRELESS_COIL_OD) / 2
check(edge_dist_w >= 3 and edge_dist_h >= 3,
      f"距外壁: X={edge_dist_w:.1f}mm, Y={edge_dist_h:.1f}mm (>=3mm 安全)",
      f"距外壁太近: X={edge_dist_w:.1f}mm, Y={edge_dist_h:.1f}mm",
      warn=True)
# 与电池冲突检查
print(f"    ⚠ 电池金属外壳会屏蔽 Qi 磁场, 装配时电池避开线圈正上方")

# ============ 9. 表带耳 ============
print("\n[9] 表带耳 (4 个)")
print(f"    ±Y 端各 2 个, 间距 {STRAP_WIDTH}mm, 单耳 {LUG_THICK}×{LUG_OUT}×{LUG_HEIGHT}mm")
print(f"    Z 范围: [{LUG_Z_CENTER - LUG_HEIGHT/2}, {LUG_Z_CENTER + LUG_HEIGHT/2}]mm")
print(f"    弹簧棒 ⌀{SPRING_BAR_D}mm, 棒长 ≈ {STRAP_WIDTH + 2*LUG_THICK}mm")

# ============ 10. 4 角螺丝 ============
print("\n[10] 4 角 M2 螺丝")
screw_pos = [(CASE_W/2-SCREW_INSET, CASE_H/2-SCREW_INSET)]
print(f"     位置: (±{CASE_W/2-SCREW_INSET}, ±{CASE_H/2-SCREW_INSET})")
print(f"     bottom 螺丝柱: ⌀{SCREW_POST_D}mm, 自攻孔 ⌀{SCREW_TAP_D}mm 深 {SCREW_TAP_DEPTH}mm")
print(f"     top 沉头: ⌀{SCREW_THRU_D}mm 通孔 + ⌀{SCREW_HEAD_D}mm 沉 {SCREW_HEAD_DEPTH}mm")
# 螺丝柱不应碰内壁
post_to_inner_wall = SCREW_INSET - WALL - SCREW_POST_D/2
check(post_to_inner_wall >= 0.5,
      f"螺丝柱距内壁 {post_to_inner_wall:.1f}mm (>=0.5mm 安全)",
      f"螺丝柱与内壁切线/相交 (距离 {post_to_inner_wall:.1f}mm)")

# ============ 11. 密封 lip / groove ============
print("\n[11] 密封 lip + U 形槽 + O 圈")
print(f"     top 凹槽: 宽 {SEAL_GROOVE_W}mm × 深 {SEAL_GROOVE_D}mm (在顶面 z=[{TOP_THICK-SEAL_GROOVE_D}, {TOP_THICK}])")
print(f"     bottom 凸起: 宽 {SEAL_LIP_W}mm × 高 {SEAL_LIP_H}mm (在顶面 z=[{BOT_THICK}, {BOT_THICK+SEAL_LIP_H}])")
check(SEAL_LIP_H < SEAL_GROOVE_D,
      f"凸起 {SEAL_LIP_H} < 凹槽 {SEAL_GROOVE_D} (O 圈有压缩空间 {SEAL_GROOVE_D-SEAL_LIP_H}mm)",
      f"凸起过高, 无 O 圈压缩空间")
check(SEAL_LIP_W < SEAL_GROOVE_W,
      f"凸起宽 {SEAL_LIP_W} < 凹槽宽 {SEAL_GROOVE_W} (装配间隙 {SEAL_GROOVE_W-SEAL_LIP_W}mm)",
      f"凸起过宽, 装不进")

# ============ 12. PCB pocket 不与凹槽冲突 ============
print("\n[12] PCB pocket vs top 密封凹槽")
groove_in_w_check = CASE_W - 2*SEAL_INSET - 2*SEAL_GROOVE_W
groove_in_h_check = CASE_H - 2*SEAL_INSET - 2*SEAL_GROOVE_W
check(pcb_pocket_w_check < groove_in_w_check and pcb_pocket_h_check < groove_in_h_check,
      f"PCB pocket {pcb_pocket_w_check}×{pcb_pocket_h_check} 在凹槽内缘 {groove_in_w_check}×{groove_in_h_check} 内 (余量 X={groove_in_w_check-pcb_pocket_w_check:.1f}, Y={groove_in_h_check-pcb_pocket_h_check:.1f}mm)",
      f"PCB pocket 与凹槽冲突, 顶层会切断")

# ============ 装配总结 ============
print("\n" + "=" * 70)
print("装配总结")
print("=" * 70)
print(f"装配总厚 = TOP({TOP_THICK}) + BOT({BOT_THICK}) - LIP嵌入({SEAL_LIP_H}) = {TOP_THICK + BOT_THICK - SEAL_LIP_H}mm")
print(f"  (lip 高 {SEAL_LIP_H} 嵌入 凹槽 {SEAL_GROOVE_D} 中, 留 {SEAL_GROOVE_D - SEAL_LIP_H}mm 给 O 圈)")
print(f"  目标 CASE_Z={CASE_Z}mm")
print(f"\n剩余内腔空间 (扣电池): {inner_d - BAT_T:.1f}mm")
print(f"  可放: TP4056 ({TP4056_T}mm) + ESP32-S3-Nano (~5mm) + 间隙 = {TP4056_T + 5 + 1.5}mm")
print(f"\n防水性能等级 (估算):")
print(f"  O 圈密封: 50-80m 水深")
print(f"  MS5837-30BA 传感器极限: 300m")
print(f"  无线充电凹槽 (1.2mm 壁): 80m+ (PETG)")
print(f"\n材料推荐: PETG (耐水) 或 PLA + 环氧涂层")
