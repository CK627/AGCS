# 搜索/逼近算法流程图（基于已清除的 search.py 实际代码）

> 依据：commit `a4f6202` 之前 `agcs_lib/search.py` 的实际实现（已核对行号）。
> 用途：重新设计搜索/逼近算法的参照图；对外接口与参数见第 4、5 节，重新实现时必须保持兼容。

## 1. 整体数据流（auto_fetch 视角）

```
tasks/auto_fetch.py main()
  │ 初始化：Board / IK / ArmIK / 相机(自检) / 红外 / 点阵 / 推流
  │ stand + 机械臂复位(21-25 reset_pulses)
  ▼
Searcher.run()
  │  search() ──找到──▶ _approach() ──面积达标──▶ 返回 (center, cy)
  │  search() ──没找到───────────────┐          │
  ▼                                  ▼          ▼
(None,None)  ──▶ 停线程+云台回中+机械臂复位+立正+退出
                                        │
                                        ▼
                              tracker.stop() → Grabber.run() → searcher.stop()
                              （夹取：前伸→占比达标→红外下降→闭夹→上抬）
```

---

## 2. search() — 扫描找目标

```
search()
  ├─ 夹爪张开(25→120)，等待 0.5s
  ├─ _home()
  │    ├─ 机械臂 setPitchRangeMoving((10,15,30), 0,-90,100, 1s)   ← 魔法数字，与 detect_pose=[0,15,5] 矛盾
  │    └─ 云台 21=500 / 24=260，等 settle(280ms)
  ├─ 原地 detect() 一次
  │    └─ 有目标? ──▶ _lock_on(r) ──确认──▶ 返回 (已锁定)
  ├─ _vertical_sweep(500)          # 21 固定 500
  │    └─ 24 从 tilt_pulses[200] 逐档到 1000
  │         └─ 每档 _smooth_tilt_to(x, 档位)：小步 20 脉宽移动 → 等到位 → detect
  │              └─ 有目标? ──▶ _lock_on ──确认──▶ 返回
  ├─ 遍历 pan_pulses=[300,200,700,900,1000]（跳过 500）
  │    ├─ 24 复位到 200
  │    ├─ _smooth_pan_to(x)：21 小步 20 脉宽移动 → 等到位 → detect → _lock_on
  │    └─ _vertical_sweep(x)
  └─ 全扫完没找到 ──▶ 返回 None
```

### 2.1 _lock_on(det) — 云台转向目标并确认

```
循环最多 12 次：
  目标中心在 x∈[200,440] 且 y∈[120,380] ?
    ├─ 是 ──▶ _confirm() 返回
    └─ 否：
        x_dis += pan_sign × 0.2 × (320 - cx)     # 水平修正（pan_sign 待定标）
        y_dis += tilt_sign × 0.2 × (240 - cy)     # 俯仰修正（tilt_sign=-1）
        发云台指令 → 等 settle(280ms) → 重新 detect
        detect 为 None ──▶ 返回 None
12 次后仍不居中 ──▶ _confirm() 兜底
```

### 2.2 _confirm(tries=4, need_hits=2) — 多帧确认

```
连续 4 帧 detect：
  中心在 x∈[200,440] 且 y∈[120,380] 才计数
≥2 帧命中 ──▶ 返回该目标
否则 ──▶ 返回 None（继续扫描）
```

---

## 3. _approach(det) — 逼近（刚重构：速度与锁定同步）

```
_approach(det)
  ├─ 启动 tracker 线程（PID 云台跟踪，后台持续 detect→修正 21/24）
  ├─ 启动避障线程（超声检测障碍 → 置停止事件）
  ├─ 转身对准：循环最多 12 次
  │    ├─ tracker.latest() 为空 → 等 50ms
  │    ├─ |21号-500| ≤ pan_band(80) → 对准，退出
  │    └─ 否则身体转 5°（方向按 21号偏移符号）→ 等 400ms
  └─ 逼近循环（最多 max_approach=12 次）：
       │
       ├─ 停止事件（避障）→ 返回 (None,None)
       ├─ _wait_target(800ms)：等 tracker 重新看到目标
       │    └─ 超时 → lost_frames≥15? "连续丢失" : "暂不可见" → 返回 (None,None)
       ├─ 取 cx,cy,dx(21号-500)
       ├─ |dx| > 80 ? → 身体转 5° → continue（本步不走）
       ├─ y 不在 [120,380]（目标被走路带偏）?
       │    ├─ _wait_centered(800ms)：等云台拉回中部
       │    │    ├─ 成功 → step_mm = 40
       │    │    └─ 超时 → _wait_target(300ms) 再确认可见
       │    │         ├─ 不可见 → 返回 (None,None)
       │    │         └─ 可见 → step_mm = 20（小步边走边校）
       │    └─ 已居中 → step_mm = 40
       ├─ 面积 ≥ approach_area_threshold(2800) ?
       │    ├─ 是 → 回中微调（身体转直到 |21-500|≤20）→ 返回 (center, cy)
       │    └─ 否 ↓
       ├─ go_forward(step_mm, 50, 1)   # 居中 40mm / 未锁准 20mm
       └─ 循环
  超步数 → 返回 (None,None)
```

---

## 4. tracker 线程（ColorTracker._run）

```
循环（每 30ms）：
  detect()
  ├─ None → _latest=None，lost_frames+1
  └─ 有目标 → _update(r)：
       x：|cx-320| ≥ dead_x(40) 才修正  x_dis += pan_sign×P×(320-cx)，否则清 PID
       y：|cy-240| ≥ dead_y(60) 才修正  y_dis += tilt_sign×P×(240-cy)，否则清 PID
       位置有变化？→ 发 21/24 指令，返回 moved
       _latest=最新目标，lost_frames=0
       moved？→ 等 settle(80ms) 再取下一帧（防运动模糊）
```

> 主线程通过 `tracker.latest()` / `lost_frames()` 读取；`stop()` 由上层调用。

---

## 5. 对外接口（重新实现时必须保持）

```python
class Searcher:
    def __init__(self, board, ik, ak, params, detect,
                 ultrasonic=None, display=None, tof=None)
    def run(self) -> (center, cy) | (None, None)   # center=(cx,cy)；cy 第二个值目前无人用
    def reset_pose(self)                            # 云台回中 21=500/24=260
    def stop(self)                                  # 停 tracker/避障线程
```

`detect` 是一个无参可调用对象，返回 `dict(center, area, radius, color, contour)` 或 None（640×480 坐标，面积 320×240 空间）。

## 6. 配置依赖（`config/robot_params.yaml`）

| 段 | 键 | 作用 |
|---|---|---|
| search | pan_pulses / tilt_pulses / tilt_scan_step / body_turn_speed | 扫描档位与步长 |
| gimbal_fetch | settle_ms / scan_move_ms / scan_settle_ms / pan_move_ms / pan_settle_ms | 扫描到位等待 |
| gimbal_fetch | tilt_sign / pan_sign / track_p_gain / track_settle_ms | 云台方向与锁定速度 |
| gimbal_fetch | dead_x/cy（track_dead_cx/cy）、pan_band、pan_band_fine、pan_turn_deg | 跟踪死区与转身 |
| gimbal_fetch | max_approach、center_wait_ms、walk_mm、walk_mm_small、walk_speed、lost_limit_frames | 逼近步进与超时 |
| obstacle | threshold / target_radius_gate | 避障 |
| grab | approach_area_threshold | 到位面积阈值（320×240 空间） |

## 7. 已知问题 / 待重新设计时决策

1. **pan_sign 未定标**：水平修正方向符号待探针实测（现在默认 +1）。
2. **tilt_sign=-1 已定标**（20:43 日志证实俯仰方向反）。
3. **_home 的 (10,15,30) 与 arm.detect_pose=[0,15,5] 矛盾**：重新设计时统一入口。
4. **run() 返回值第二个元素 cy 无人用**：可简化为只返回 center。
5. **扫描顺序**：500 → 300 → 200 → 700 → 900 → 1000（先左后右，几何上可能漏掉初始方位的目标，重新设计可考虑从 500 向两侧交替扩展）。
6. **`_smooth_*` 扫描与 `_lock_on` 职责重叠**：可合并为单一"扫描-确认"状态。
7. 已清除的旧实现可通过 git 历史恢复（commit `a4f6202` 之前的版本）。
