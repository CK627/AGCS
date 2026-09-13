# FixedGripTest 使用说明

固定夹取/放下流程测试目录。

## 目录文件

```text
record_route.py          手动录制固定路线
manual_control.py        手动控制六足，不记录
full_loop_test.py        只走完整路线，跳过夹取/放下
return_origin.py         按录制路线反向回原点
NO1.py                   固定路线 + 固定夹取
capture_route_points.py  自动走点并标定夹取/放下位置
fixed_route.json         录制的路线
success_points.json      标定成功的夹取/放下点
```

## 开始前

```bash
ssh pi@10.194.228.89
sudo systemctl stop spiderpi
cd /home/pi/spiderpi
```

## 1. 录制路线

```bash
python3 tasks/ceshi/FixedGripTest/record_route.py
```

默认参数：

```text
直线步长：50
转弯角度：10
速度：50
```

可覆盖：

```bash
python3 tasks/ceshi/FixedGripTest/record_route.py --step 50 --angle 10 --speed 50
```

### 录制脚本常用按键

| 按键 | 动作 |
|---|---|
| `w` | 前进一步 |
| `s` | 后退一步 |
| `a` | 左横移一次 |
| `d` | 右横移一次 |
| `q` | 左转 10 度 |
| `e` | 右转 10 度 |
| `f` | 标记夹取点 |
| `g` | 标记放下点 |
| `r` | 恢复立正并记录 stand |
| `u` | 撤销上一步并反向执行 |
| `c` | 清空全部记录并恢复立正 |
| `l` | 查看当前记录 |
| `x` | 保存到 fixed_route.json 并退出 |

### 临时参数命令

输入冒号命令后按回车：

| 命令 | 含义 |
|---|---|
| `:w50` | 临时前进一步，步长 50 |
| `:s50` | 临时后退一步，步长 50 |
| `:a50` | 临时左横移一次，步长 50 |
| `:d50` | 临时右横移一次，步长 50 |
| `:q10` | 临时左转 10 度 |
| `:e10` | 临时右转 10 度 |

## 2. 手动控制六足

```bash
python3 tasks/ceshi/FixedGripTest/manual_control.py
```

按键和录制脚本相同，但不会记录路线。

## 3. 只走完整路线测试

```bash
python3 tasks/ceshi/FixedGripTest/full_loop_test.py
```

会按 `fixed_route.json` 走完整路线，遇到 `pick` / `place` 跳过，不执行机械臂。

## 4. 自动走点并标定夹取/放下

```bash
python3 tasks/ceshi/FixedGripTest/capture_route_points.py
```

### 夹取点 pick

1. 机器人停下
2. 手动用 App 调机械臂
3. 按 `f` 执行夹取，25 固定为 700
4. 观察结果：
   - 第一次成功输入 `:f1`
   - 第二次成功输入 `:f2`
   - 失败按 `u` 重试

### 放下点 place

1. 机器人停下
2. 手动用 App 调机械臂
3. 按 `g` 执行放下，25 固定为 400
4. 观察结果：
   - 第一次成功输入 `:g1`
   - 第二次成功输入 `:g2`
   - 失败按 `u` 重试

### 标定脚本常用按键

| 输入 | 含义 |
|---|---|
| `f` | 在 pick 点执行夹取并读取 22-23-24-21 |
| `g` | 在 place 点执行放下并读取 22-23-24-21 |
| `:f1` | 保存第一次夹取成功 |
| `:f2` | 保存第二次夹取成功 |
| `:g1` | 保存第一次放下成功 |
| `:g2` | 保存第二次放下成功 |
| `u` | 当前失败，恢复官方初始位置并重试 |

机械臂读取/恢复顺序固定为：

```text
22 → 23 → 24 → 21
```

成功位置保存到：

```text
/home/pi/spiderpi/tasks/ceshi/FixedGripTest/success_points.json
```
