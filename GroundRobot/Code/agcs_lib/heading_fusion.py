#!/usr/bin/python3
# coding=utf8
"""航向/横向偏差融合估计：把 IMU 与相机从「互相拉扯」改成「各司其职」。

## 现状的两个回路为什么会打架

IMU 回路管航向（turn），颜色回路管横向（left/right_move），看起来各管一摊，
实际上它们被**同一台机器人**强耦合在一起：

1. **相机装在机身上。** 机身转了 e 度，画面里色块就横移 `f·tan(e)` 像素 ——
   这和「机器人横移了 d 毫米」在画面上**完全无法区分**。
2. **平移修不掉航向。** 于是真实链路是：航向歪 → 色块偏移 → 发平移（航向没变）
   → 往前走一段又偏出去 → 再平移……机器人走的是斜线，航向误差一直躺在那儿。
   这就是现场看到的「先直、然后一路偏出去」。
3. **横向平移不是纯平移。** 六足 crab 走一步基本都会踢一下航向 —— 颜色回路在往
   IMU 回路里注入扰动，IMU 再转回来，两个回路开始对打。日志上就表现为
   「IMU 数值乱」。

## 为什么「纠偏后重置 IMU」治标不治本

单次方位观测只给出 **1 个约束**，却要定 **2 个未知数**（航向误差 e、横向偏差 cross）：

    β = atan2(−cross, R) − e

能把这两个量分开的是**时间**：机器人往前走，R 变小，β 对 cross 的敏感度上升、
对 e 的敏感度不变 —— 正是这段「边走边看」的过程把两个状态分开。

而 `tracker.reset()` 每转一次弯就把 e 的积分清零，等于把好不容易攒下来的
可观测性丢掉；紧接着相机那边 `color_state['ref_cx'] = None` 又重新取基准 ——
**两个绝对参考同时归零**，系统从此再没有任何东西知道「我到底歪了多少」。

所以：reset 不是让误差消失，是**把误差从 `yaw` 这个数字里搬到了机器人的真实姿态里**。
数字干净了，机器人歪了。

## 本模块的做法

把 IMU 当**预测**（短期准、长期漂），把相机当**观测**（绝对、慢、会丢帧），
两者只通过一个**不归零的状态** `(e, cross)` 交流：

    IMU Δθ  ──► 预测 ──┐
                       ├─► 融合状态 (e, cross) ──► 单一控制器（先航向后横向）
    相机 β  ──► 观测 ──┘

规则：**相机写状态，IMU 管执行。** 谁也不去「消掉对方造成的结果」。
bias 标定照做（那是传感器属性），但**状态永不清零**。

## 坐标系约定

- `e`     ：相对本段标称航向的偏差，**度，右正**
- `cross` ：相对标称中心线的横向偏差，**毫米，右正**
- `β`     ：色块相对相机光轴的方位角，度，右正；`β = atan((cx − cx0) / f_px)`
- `R`     ：到色块的前向距离，毫米

纯 Python 实现（不依赖 numpy），树莓派上可直接跑。
"""

import math

_DEG = math.pi / 180.0
_RAD = 180.0 / math.pi


def pixel_to_bearing_deg(cx, cx0, f_px):
    """像素横坐标 → 相对相机光轴的方位角（度，右正）。"""
    return math.atan((float(cx) - float(cx0)) / float(f_px)) * _RAD


def range_from_radius_px(radius_px, real_radius_mm, f_px):
    """用色块视半径反推前向距离（毫米）。real_radius_mm 是色块的真实半径。"""
    if radius_px is None or radius_px <= 0 or real_radius_mm <= 0:
        return None
    return float(f_px) * float(real_radius_mm) / float(radius_px)


class LaneFusion(object):
    """(航向误差 e, 横向偏差 cross) 的两状态卡尔曼融合。

    predict：IMU 测到的航向增量 + 本段前进距离（航向误差会转化成横向增长）
    update ：相机测到的色块方位角 β

    用法（每个 100mm 小块一个循环）：:

        f = LaneFusion(f_px=277.0, cx0=160.0)
        while 前进中:
            f.predict(d_theta_imu, ds_mm=100.0, lateral_mm=0.0)
            if 看到色块:
                f.update_bearing(cx_px, range_mm)
            turn, lateral = ctrl.decide(f.heading_error, f.cross_error)
    """

    def __init__(self, f_px=277.0, cx0=160.0,
                 q_head=0.05, q_cross=1.5, r_beta=0.03,
                 cross_limit=800.0):
        # 相机内参：焦距 f_px 与主点 cx0，单位都是「detect_color 缩放后的像素」
        # （detect_color 内部 resize 到 320×240，所以 bbox_center_x 是 320 坐标系）
        self.f_px = float(f_px)
        self.cx0 = float(cx0)
        # 过程噪声：q_head 是每 100mm 步态带来的未建模航向扰动（度²），
        # q_cross 是横向滑移（mm²）。调大 = 更信任相机。
        self.q_head = float(q_head)
        self.q_cross = float(q_cross)
        # 观测噪声：方位角测量方差（度²）。色块越小/越晃就调大。
        self.r_beta = float(r_beta)
        self.cross_limit = abs(float(cross_limit))

        self.e = 0.0        # 航向误差估计（度，右正）
        self.cross = 0.0    # 横向偏差估计（mm，右正）
        self.p00 = 4.0      # 协方差，初值偏大 = 一开始更信任相机
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 400.0
        self.updates = 0    # 收到过多少次相机观测

    # ---------- 状态 ----------
    @property
    def heading_error(self):
        """航向误差估计，度，右正。控制器要的就是这个。"""
        return self.e

    @property
    def cross_error(self):
        """横向偏差估计，mm，右正。"""
        return self.cross

    def set_state(self, e=0.0, cross=0.0, keep_cov=False):
        """只在「标称方向真的变了」时才用（例如换了一段新路线的起点）。

        注意：**不要**在每个直行段开头调用它，那和 reset IMU 是同一个错误。
        """
        self.e = float(e)
        self.cross = float(cross)
        if not keep_cov:
            self.p00, self.p01, self.p10, self.p11 = 1.0, 0.0, 0.0, 100.0

    # ---------- 预测 ----------
    def predict(self, d_theta_deg, ds_mm=0.0, lateral_mm=0.0):
        """IMU 增量 + 位移推进一步。

        d_theta_deg：这一段 IMU 测到的航向增量（度，右正）
        ds_mm      ：这一段前进的距离（mm），航向误差会按 ds·sin(e) 转成横向增长
        lateral_mm ：这一段主动横移的距离（mm，右正）
        """
        self.e += float(d_theta_deg)
        # 航向误差 → 横向漂移：这是「不纠航向就一定会偏出去」的数学来源
        self.cross += float(ds_mm) * math.sin(self.e * _DEG) + float(lateral_mm)
        if self.cross > self.cross_limit:
            self.cross = self.cross_limit
        elif self.cross < -self.cross_limit:
            self.cross = -self.cross_limit

        # 雅可比 F = [[1, 0], [a, 1]]，a = ∂cross/∂e（mm/deg）
        a = float(ds_mm) * math.cos(self.e * _DEG) * _DEG
        p00, p01, p10, p11 = self.p00, self.p01, self.p10, self.p11
        self.p00 = p00
        self.p01 = p00 * a + p01
        self.p10 = a * p00 + p10
        self.p11 = (a * p00 + p10) * a + (a * p01 + p11)
        # 对称化，防长时间运行的数值漂移
        self.p01 = self.p10 = (self.p01 + self.p10) * 0.5

        self.p00 += self.q_head
        self.p11 += self.q_cross

    # ---------- 观测更新 ----------
    def update_bearing(self, cx_px, range_mm):
        """用色块像素横坐标做一次观测更新。返回更新后的残差（度），看不到就返回 None。"""
        if cx_px is None or range_mm is None or range_mm <= 1.0:
            return None
        beta = pixel_to_bearing_deg(cx_px, self.cx0, self.f_px)
        return self.update_bearing_deg(beta, range_mm)

    def update_bearing_deg(self, beta_deg, range_mm):
        """直接用方位角（度）做观测更新，返回残差（度）。"""
        r = float(range_mm)
        # h = atan2(-cross, R) - e
        h = math.atan2(-self.cross, r) * _RAD - self.e
        # 残差归一化到 (-180, 180]
        y = (float(beta_deg) - h + 180.0) % 360.0 - 180.0

        # 雅可比 H = [∂h/∂e, ∂h/∂cross] = [-1, -R/(cross²+R²)·(180/π)]
        h0 = -1.0
        h1 = -r / (self.cross * self.cross + r * r) * _RAD

        p00, p01, p10, p11 = self.p00, self.p01, self.p10, self.p11
        ph0 = p00 * h0 + p01 * h1      # (P Hᵀ)[0]
        ph1 = p10 * h0 + p11 * h1      # (P Hᵀ)[1]
        s = h0 * ph0 + h1 * ph1 + self.r_beta
        if s <= 1e-9:
            return y
        k0 = ph0 / s
        k1 = ph1 / s

        self.e += k0 * y
        self.cross += k1 * y
        if self.cross > self.cross_limit:
            self.cross = self.cross_limit
        elif self.cross < -self.cross_limit:
            self.cross = -self.cross_limit

        n00 = 1.0 - k0 * h0
        n01 = -k0 * h1
        n10 = -k1 * h0
        n11 = 1.0 - k1 * h1
        self.p00 = n00 * p00 + n01 * p10
        self.p01 = n00 * p01 + n01 * p11
        self.p10 = n10 * p00 + n11 * p10
        self.p11 = n10 * p01 + n11 * p11
        self.p01 = self.p10 = (self.p01 + self.p10) * 0.5

        self.updates += 1
        return y

    # ---------- 诊断 ----------
    def solve_cross_mm(self, beta_deg, range_mm):
        """给定当前航向估计，把方位角反解成横向偏差（mm）。用于对照/调试。"""
        return -float(range_mm) * math.tan((float(beta_deg) + self.e) * _DEG)

    def __repr__(self):
        return ('LaneFusion(e=%+.2fdeg cross=%+.1fmm P=[%.2f,%.1f] n=%d)'
                % (self.e, self.cross, self.p00, self.p11, self.updates))


class LaneController(object):
    """单一控制器：**先纠航向，再纠横向**，同一小块里绝不同时发两种指令。

    这是解耦的另一半：原来两个回路各自盯着自己的误差、各用自己的通道，
    现在的规则是一条 —— 航向误差是横向误差的**因**，因没除掉之前不动果。
    """

    def __init__(self, head_deadzone=1.0, head_gain=0.9, head_max=6.0,
                 cross_deadzone=8.0, cross_gain=0.35, cross_max=25.0):
        self.head_deadzone = float(head_deadzone)
        self.head_gain = float(head_gain)
        self.head_max = float(head_max)
        self.cross_deadzone = float(cross_deadzone)
        self.cross_gain = float(cross_gain)
        self.cross_max = float(cross_max)
        self.last_action = 'none'

    def decide(self, e_deg, cross_mm):
        """返回 (turn_deg, lateral_mm)。

        turn_deg   ：>0 右转，<0 左转
        lateral_mm ：>0 右移，<0 左移
        两者至多一个非零。
        """
        if abs(e_deg) > self.head_deadzone:
            turn = -self.head_gain * e_deg          # 朝右歪（e>0）→ 左转（负）
            turn = max(-self.head_max, min(self.head_max, turn))
            # 最小步进：小于 1° 的转向六足基本执行不出来，攒着
            if abs(turn) < 1.0:
                turn = math.copysign(1.0, turn)
            self.last_action = 'turn'
            return turn, 0.0

        if abs(cross_mm) > self.cross_deadzone:
            lat = -self.cross_gain * cross_mm       # 偏右（cross>0）→ 左移（负）
            lat = max(-self.cross_max, min(self.cross_max, lat))
            if abs(lat) < 3.0:
                lat = math.copysign(3.0, lat)
            self.last_action = 'lateral'
            return 0.0, lat

        self.last_action = 'none'
        return 0.0, 0.0


class ReferenceFusion(object):
    """按路线走、方块当固定参照的融合（B 方案）。

    与 LaneFusion 的本质区别只在**观测基准**：
    - LaneFusion 假设方块在正前方（或把初始方位冻成 cx0），机器人前进时方块「应有
      的」方位几何变化被当成漂移 → 机器人被「吸」向方块（现场「一路往外走」根因）；
    - 本类跟踪方块在理想路线里的位置 (bx, by)，把「方块应有的方位变化」β_ref 从
      观测里扣掉，剩下的才是漂移 → 机器人照路线走，方块只用来纠漂移。

    状态与 LaneFusion **完全相同**：(e 航向误差度, cross 横向偏差 mm)，用**同一套
    卡尔曼**（不是瞬时相减）。多出来的 (bx, by) 只是确定性的方块位置预测（按命令
    运动推进），不进卡尔曼状态。

    上一版在现场发散（e 飙到 -142°）的根因：
    1. update 直接 `e = β_pred - β_meas`，瞬时值 + 控制器转向 = 正反馈；
    2. predict 用「实际 IMU 转角」转 (bx, by)，漂移被转没了、又跟噪声/延迟打架。
    这版改成卡尔曼平滑，predict 的转角拆成「命令（推进方块）/ 残差（累积 e）」两个。

    坐标约定：
    - e     ：航向误差，度，右正（相对理想路线航向）
    - cross ：横向偏差，mm，右正（相对路线中心线）
    - bx/by ：方块在理想路线系里的位置，mm；bx 右正、by 前正
    - β_ref ：方块应有方位 = atan2(bx, by)
    """

    def __init__(self, f_px=419.0, cx0=160.0,
                 q_head=0.05, q_cross=1.5, r_beta=0.03, cross_limit=800.0,
                 min_ahead_mm=200.0, max_innov_deg=30.0):
        self.f_px = float(f_px)
        self.cx0 = float(cx0)
        self.q_head = float(q_head)
        self.q_cross = float(q_cross)
        self.r_beta = float(r_beta)
        self.cross_limit = abs(float(cross_limit))
        # 方块前向距离小于这个值（太近/已到身侧）时，方位基准 atan2(bx,by) 对误差极敏感，
        # 观测不可靠，跳过本次更新，冻结状态靠 IMU 短期精度继续走。
        self.min_ahead_mm = float(min_ahead_mm)
        # 创新门限（度）：单次观测残差超过它多半是野值/模型失配，直接丢弃，防止发散。
        self.max_innov_deg = float(max_innov_deg)

        self.e = 0.0         # 航向误差（度，右正）
        self.cross = 0.0     # 横向偏差（mm，右正）
        self.p00 = 4.0       # 协方差，初值偏大 = 一开始更信任相机
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 400.0
        self.updates = 0     # 收到过多少次相机观测

        self.bx = 0.0        # 方块横向位置（mm，右正，理想路线系）
        self.by = 0.0        # 方块前向位置（mm，前正，理想路线系）
        self.initialized = False

    @property
    def heading_error(self):
        return self.e

    @property
    def cross_error(self):
        return self.cross

    def init(self, cx_px, range_mm):
        """第一次看到方块：由方位角 + 距离定出方块在理想路线系里的位置。"""
        beta = math.atan((cx_px - self.cx0) / self.f_px)
        self.bx = range_mm * math.sin(beta)
        self.by = range_mm * math.cos(beta)
        self.initialized = True

    def predict(self, d_theta_deg, ds_mm=0.0, lateral_mm=0.0,
                d_theta_cmd_deg=None):
        """推进一步（与 LaneFusion.predict 同签名，多一个命令转角）。

        d_theta_deg     ：实际转角（直行段 = IMU 实测漂移），或转弯时的残差（实际−命令）。
                         用它累积卡尔曼状态 e（IMU 短期准、长期靠方块纠）。
        d_theta_cmd_deg ：命令转角，只用来推进方块位置 (bx, by)。缺省=0（直行段不转）。
        ds_mm / lateral_mm：本段前进 / 横移（命令值）。
        """
        cmd = 0.0 if d_theta_cmd_deg is None else float(d_theta_cmd_deg)

        # (1) 方块位置按「命令运动」推进：直行只前移、不转，所以方块「应有的」方位
        #     变化被完整保留，不会被误判成漂移。
        if self.initialized:
            if cmd:
                th = math.radians(cmd)
                bx = self.bx * math.cos(th) - self.by * math.sin(th)
                by = self.bx * math.sin(th) + self.by * math.cos(th)
                self.bx, self.by = bx, by
            self.by -= float(ds_mm)
            self.bx -= float(lateral_mm)

        # (2) 卡尔曼状态推进（与 LaneFusion 完全相同）
        self.e += float(d_theta_deg)
        self.cross += float(ds_mm) * math.sin(self.e * _DEG) + float(lateral_mm)
        if self.cross > self.cross_limit:
            self.cross = self.cross_limit
        elif self.cross < -self.cross_limit:
            self.cross = -self.cross_limit

        a = float(ds_mm) * math.cos(self.e * _DEG) * _DEG
        p00, p01, p10, p11 = self.p00, self.p01, self.p10, self.p11
        self.p00 = p00
        self.p01 = p00 * a + p01
        self.p10 = a * p00 + p10
        self.p11 = (a * p00 + p10) * a + (a * p01 + p11)
        self.p01 = self.p10 = (self.p01 + self.p10) * 0.5
        self.p00 += self.q_head
        self.p11 += self.q_cross

    def update(self, cx_px, range_mm):
        """用方块实测方位做观测更新，返回 (heading_error_deg, cross_mm)。

        观测是「实测方位 − 方块应有方位」β_rel，卡尔曼模型 h = atan2(−cross, R) − e
        正是预测的 β_rel（cross/e 全 0 时 β_rel 应为 0），与 LaneFusion 同一个模型，
        只是把 LaneFusion 里被 cx0 冻死的基准换成了会随接近而演化的 β_ref。
        """
        if cx_px is None or range_mm is None or range_mm <= 1.0:
            return self.e, self.cross
        if not self.initialized:
            self.init(cx_px, range_mm)
            return self.e, self.cross

        # 方块太近（即将到达身侧/身后）时，方位基准失效：atan2(bx, by) 对 by→0 极敏感，
        # 会把小误差放大成几十度的假残差（现场 e 从 3° 跳到 149° 就是这里）。跳过。
        if self.by < self.min_ahead_mm:
            return self.e, self.cross

        beta_meas = math.atan((cx_px - self.cx0) / self.f_px) * _RAD   # 度
        beta_ref = math.atan2(self.bx, self.by) * _RAD                 # 度
        beta_rel = (beta_meas - beta_ref + 180.0) % 360.0 - 180.0

        # 用跟踪到的前向距离 by（与 beta_ref 一致），不再用冻结的 marker_range，
        # 否则越接近方块，模型 h 和基准 beta_ref 越对不上。
        r = self.by
        h = math.atan2(-self.cross, r) * _RAD - self.e                 # 度
        y = (beta_rel - h + 180.0) % 360.0 - 180.0
        # 创新门限：单次观测残差超阈值当野值丢掉，别让它把状态抽爆
        if abs(y) > self.max_innov_deg:
            return self.e, self.cross

        h0 = -1.0
        h1 = -r / (self.cross * self.cross + r * r) * _RAD
        p00, p01, p10, p11 = self.p00, self.p01, self.p10, self.p11
        ph0 = p00 * h0 + p01 * h1
        ph1 = p10 * h0 + p11 * h1
        s = h0 * ph0 + h1 * ph1 + self.r_beta
        if s <= 1e-9:
            return self.e, self.cross
        k0 = ph0 / s
        k1 = ph1 / s

        self.e += k0 * y
        self.cross += k1 * y
        if self.cross > self.cross_limit:
            self.cross = self.cross_limit
        elif self.cross < -self.cross_limit:
            self.cross = -self.cross_limit

        n00 = 1.0 - k0 * h0
        n01 = -k0 * h1
        n10 = -k1 * h0
        n11 = 1.0 - k1 * h1
        self.p00 = n00 * p00 + n01 * p10
        self.p01 = n00 * p01 + n01 * p11
        self.p10 = n10 * p00 + n11 * p10
        self.p11 = n10 * p01 + n11 * p11
        self.p01 = self.p10 = (self.p01 + self.p10) * 0.5

        self.updates += 1
        return self.e, self.cross

    def update_bearing(self, cx_px, range_mm):
        """与 LaneFusion.update_bearing 同名的观测入口，方便 1.py 无差别调用。"""
        return self.update(cx_px, range_mm)

    def reset(self):
        """夹取/放下后调用：方块已被抓走/放下，参照失效，复位等下一个方块重新初始化。

        不 reset 的话，旧方块位置 (bx,by) 还留在状态里，下一段直线若再检测到（夹爪里
        的方块 / 别的红目标），会把「实测方位 − 旧参照方位」当成巨大漂移 → e 又飙。
        """
        self.initialized = False
        self.e = 0.0
        self.cross = 0.0
        self.p00, self.p01, self.p10, self.p11 = 4.0, 0.0, 0.0, 400.0
        self.bx = 0.0
        self.by = 0.0
        self.updates = 0

    def __repr__(self):
        return ('ReferenceFusion(e=%+.2fdeg cross=%+.1fmm block=(%+.0f,%+.0f) n=%d)'
                % (self.e, self.cross, self.bx, self.by, self.updates))
