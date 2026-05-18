// =====================================================================
// DiveWatch 防水外壳 - 紧凑型 60×65×22mm
// 适用: ESP32-S3-Nano + MS5837-30BA + OLED SH1106 + 3 按钮 + 电池
// 打印: 拓竹 PLA / 推荐 PETG (耐水)
// =====================================================================

// ============== 参数 (可调) ==============
$fn = 64;                  // 圆形精度

// 外壳总尺寸
CASE_W      = 60;          // 宽
CASE_H      = 65;          // 高
CASE_D      = 22;          // 厚
WALL        = 2.0;         // 壁厚
TOP_H       = 12;          // 上盖高度
BOT_H       = 10;          // 下底高度
CORNER_R    = 4;           // 外圆角

// OLED 显示窗口 (镂空)
OLED_W      = 27;          // 可视宽 (SH1106 1.3" 实际 ~25, 留 1mm 边)
OLED_H      = 17;          // 可视高
OLED_OFFSET_Y = 8;         // 距上沿距离

// 按钮 (3 个, 左侧从上到下: MODE/UP/DOWN)
BTN_HOLE_D  = 6.5;         // 按钮压杆通孔
BTN_SPACING = 12;          // 按钮中心间距
BTN_TOP_OFF = 20;          // 第一个按钮距顶部距离

// MS5837 传感器孔 (底部中央, 接触水)
MS_HOLE_D   = 4.0;         // 传感器圆顶外径
MS_GROOVE_D = 7.0;         // 外圈 O 圈槽直径

// USB-C 充电口 (TP4056 输出, 右侧)
USBC_W      = 9.5;         // USB-C 母座宽
USBC_H      = 4.5;         // USB-C 母座高

// 蜂鸣器透音孔 (顶面右上, 内贴 GORE-TEX 防水膜)
BUZZ_HOLE_D = 1.5;
BUZZ_OFFSET = 8;           // 距右上角距离

// 螺丝孔 (M2 × 6mm 不锈钢)
SCREW_D     = 2.2;         // 过孔
SCREW_HEAD  = 4.0;         // 沉头孔直径
SCREW_INSET = 4.5;         // 距边距离

// O 圈密封槽 (上下盖之间)
ORING_W     = 1.8;         // O 圈截面直径
ORING_DEPTH = 1.2;         // 槽深 (略小于 O 圈让其挤压密封)

// 表带耳 (两侧, 22mm 标准表带)
STRAP_W     = 22;          // 表带宽
LUG_D       = 2.5;         // 弹簧棒直径

// ============== 工具函数 ==============

// 圆角矩形长方体
module rounded_box(w, h, d, r) {
    hull() {
        for (x = [-1, 1], y = [-1, 1]) {
            translate([x*(w/2-r), y*(h/2-r), 0])
                cylinder(r=r, h=d);
        }
    }
}

// 倒圆角空腔 (内部, 较小圆角)
module inner_cavity(w, h, d, r) {
    translate([0, 0, WALL])
        rounded_box(w - 2*WALL, h - 2*WALL, d - WALL, max(r - WALL, 1));
}

// ============== 上盖 ==============
module top_cover() {
    difference() {
        union() {
            // 主体
            rounded_box(CASE_W, CASE_H, TOP_H, CORNER_R);
            // 表带耳 (左右两侧)
            for (side = [-1, 1])
                translate([side*(CASE_W/2), 0, TOP_H/2])
                    rotate([90, 0, 0])
                        cylinder(d = 6, h = STRAP_W, center=true);
        }

        // 内腔
        inner_cavity(CASE_W, CASE_H, TOP_H, CORNER_R);

        // OLED 显示窗口
        translate([0, -(CASE_H/2 - OLED_OFFSET_Y - OLED_H/2), TOP_H - WALL/2])
            cube([OLED_W, OLED_H, WALL + 1], center=true);

        // 3 按钮通孔 (左侧, 顶面钻入)
        for (i = [0:2]) {
            translate([-(CASE_W/2 - 6), -(CASE_H/2 - BTN_TOP_OFF - i*BTN_SPACING),
                       TOP_H - WALL/2])
                cylinder(d = BTN_HOLE_D, h = WALL + 1, center=true);
        }

        // 蜂鸣器透音孔 (右上)
        translate([CASE_W/2 - BUZZ_OFFSET, CASE_H/2 - BUZZ_OFFSET, TOP_H - WALL/2])
            cylinder(d = BUZZ_HOLE_D, h = WALL + 1, center=true);

        // 4 颗螺丝沉头孔 (从顶面往下)
        for (x = [-1, 1], y = [-1, 1]) {
            translate([x*(CASE_W/2 - SCREW_INSET), y*(CASE_H/2 - SCREW_INSET), 0]) {
                // 通孔
                cylinder(d = SCREW_D, h = TOP_H + 1);
                // 沉头
                translate([0, 0, TOP_H - 2])
                    cylinder(d = SCREW_HEAD, h = 2.5);
            }
        }

        // 表带耳弹簧棒孔 (穿透 22mm)
        for (side = [-1, 1])
            translate([side*(CASE_W/2 + 0.01), 0, TOP_H/2])
                rotate([90, 0, 0])
                    cylinder(d = LUG_D, h = STRAP_W + 0.5, center=true);

        // O 圈密封槽 (底面边缘内圈)
        translate([0, 0, -0.01])
            difference() {
                rounded_box(CASE_W - 2.5, CASE_H - 2.5, ORING_DEPTH, CORNER_R - 1.5);
                rounded_box(CASE_W - 2.5 - 2*ORING_W, CASE_H - 2.5 - 2*ORING_W,
                            ORING_DEPTH + 0.1, CORNER_R - 1.5 - ORING_W);
            }
    }
}

// ============== 下底 ==============
module bottom_case() {
    difference() {
        union() {
            rounded_box(CASE_W, CASE_H, BOT_H, CORNER_R);
            // 表带耳
            for (side = [-1, 1])
                translate([side*(CASE_W/2), 0, BOT_H/2])
                    rotate([90, 0, 0])
                        cylinder(d = 6, h = STRAP_W, center=true);
        }

        // 内腔
        inner_cavity(CASE_W, CASE_H, BOT_H, CORNER_R);

        // MS5837 传感器孔 (底部中央)
        translate([0, 0, 0])
            cylinder(d = MS_HOLE_D, h = WALL + 1, center=true);

        // MS5837 O 圈槽 (外圈)
        translate([0, 0, WALL/2])
            difference() {
                cylinder(d = MS_GROOVE_D, h = ORING_DEPTH);
                cylinder(d = MS_GROOVE_D - 2*ORING_W, h = ORING_DEPTH + 0.1);
            }

        // USB-C 充电口 (右侧)
        translate([CASE_W/2, -CASE_H/4, BOT_H/2 + 1])
            cube([WALL + 2, USBC_W, USBC_H], center=true);

        // 螺丝攻丝孔 (M2 自攻, 内径稍小)
        for (x = [-1, 1], y = [-1, 1]) {
            translate([x*(CASE_W/2 - SCREW_INSET), y*(CASE_H/2 - SCREW_INSET), 0])
                cylinder(d = SCREW_D - 0.3, h = BOT_H + 0.5);
        }

        // 表带耳孔
        for (side = [-1, 1])
            translate([side*(CASE_W/2 + 0.01), 0, BOT_H/2])
                rotate([90, 0, 0])
                    cylinder(d = LUG_D, h = STRAP_W + 0.5, center=true);
    }
}

// ============== 渲染选择 ==============
// 取消注释你要打印的部分

// 上盖
top_cover();

// 下底 (取消注释来渲染)
// translate([CASE_W + 10, 0, 0])
//     bottom_case();

// 同时显示两半 (装配视图)
// top_cover();
// translate([0, 0, -BOT_H - 0.5]) bottom_case();
