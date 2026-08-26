# YOLO 模型训练教程（Python 零基础版）

> 这份教程是给**完全没学过 Python** 的队友写的。
> 讲代码的方式：**先讲整段在干嘛，复杂的地方才拆开讲**，不再一行一行拆。
>
> 先记 5 个比喻，全书都用它们：
>
> - **工具箱**＝Python 里别人做好的现成工具集合（写代码时用 `import` 打开）；
> - **工具**＝函数，一段起好名字、能反复使用的步骤；
> - **旋钮**＝参数，使用工具前要设置的选项；
> - **盒子**＝变量，一个贴了名字、用来存东西的地方；
> - **便利贴**＝注释，`#` 开头，写给人看，电脑直接跳过。
>
> 配套主文档：[YOLO教学与空地协同.md](../YOLO教学与空地协同.md)；
> macOS 训练：[MacYoLo/README.md](../MacYoLo/README.md)。
>
> 本文按 **Windows 地面站（机械革命 + RTX 5060）** 编写：YOLO 工作目录统一用
> `D:\yolo`，运行命令用 `python`（不是 `python3`），显卡参数用 `device=0`。
> 环境安装（NVIDIA 驱动、torch `+cu128`）见 [README.md](../README.md) 2.3 节。

---

## 0. 先花 15 分钟认识 Python

### 0.1 程序是什么

程序就是一份**菜谱**：一行一行告诉电脑先做什么、再做什么。电脑很听话但很笨，
不会猜你的意思，所以每一步都要写清楚。

### 0.2 工具箱（库）和 import（打开工具箱）

Python 有很多别人做好的**工具箱**，里面装着现成工具，不用自己造。用
`import` 打开：

```python
import cv2      # 打开"图像工具箱"（读视频、存图片、显示画面）
import os       # 打开"电脑文件工具箱"（管文件夹、文件名）
```

打开之后，用 `工具箱名.工具名` 拿工具，比如 `cv2.VideoCapture(...)` 就是
"用图像工具箱里的 VideoCapture 工具"。

### 0.3 工具（函数）和旋钮（参数）

函数就是**工具**。拿榨汁机举例：工具名 `榨汁`，旋钮是"放什么水果、要不要
去核"，按下开关（使用工具）得到果汁（交回的结果）。

```python
cap = cv2.VideoCapture('视频.mp4')
```

意思：用图像工具箱的 VideoCapture 工具，旋钮设为 `'视频.mp4'`，把得到的
"视频读取器"放进叫 `cap` 的盒子里。

### 0.4 盒子（变量）

`盒子名 = 值` 就是往盒子里放东西：

```python
n = 0          # 一个叫 n 的盒子，里面放数字 0
```

`n = n + 1` ＝ 把 n 盒子里的数拿出来加 1 再放回去；简写 `n += 1` 完全一样。

### 0.5 便利贴（注释）

`#` 开头的内容是便利贴，电脑直接跳过：

```python
# 这是给队友看的说明，电脑不理它
n += 1         # 帧号加 1
```

三个引号 `"""..."""` 也是便利贴，只是可以写好几行，一般放在文件开头说明
"这个文件是干嘛的"。

### 0.6 一排格子（列表）和带名字的柜子（字典）

- **一排格子** `[1, 2, 3]`：用编号取，`格子[0]` 是第 1 个（**编号从 0 数起**）；
- **带名字的柜子** `{'名字': '小明', '年龄': 18}`：用名字取，
  `柜子['名字']` 得到 `'小明'`。

代码里 `{'center': (10, 20), 'area': 500}` 就是一个柜子：一个格子叫 center，
一个叫 area。

### 0.7 反复做（循环）

- `while True:` ＝ 一直做下面缩进的事，直到遇到 `break`（喊停）；
- `for x in 一串东西:` ＝ 把这串东西一个一个拿出来，每次叫 x，做一遍缩进的事。

```python
while True:
    ok, frame = cap.read()   # 读一帧
    if not ok:
        break                # 读不到了，喊停
```

### 0.8 岔路口（判断）

`if 条件:` 像走到岔路口，条件成立才走这条路：

```python
if n % 15 == 0:      # 如果 n 是 15 的倍数（% 是"取余数"，10 % 3 = 1）
    print('取这张')
```

### 0.9 缩进（分组）—— Python 最重要的规矩

行首的空格表示"这些行属于上面那句"：

```python
if 条件:
    属于这里的第 1 行
    属于这里的第 2 行
不属于这里了（顶格写）
```

缩进错了程序直接报错。看到 `IndentationError` 就是缩进问题。

### 0.10 自己造工具（def）和"只有直接运行才执行"

`def 名字(旋钮):` 是造一个新工具，下面缩进的都是它的步骤：

```python
def extract_frames(video_path, step=15):
    # 这里写"提取帧"工具的步骤
    ...
```

`if __name__ == '__main__':` 记住一句话就行：**"如果我是直接运行这个文件，
就执行下面的事；如果我是被别的文件拿去当工具箱用，就不执行。"**

### 0.11 终端 / 命令行

终端是"用文字给电脑下命令"的地方：

```bash
cd 文件夹                # 进入这个文件夹
python 文件名.py          # 让 Python 运行这个文件
python 文件名.py --data 数据.yaml   # 运行文件，并传两个设置
```

`--data` 是设置的名字，`数据.yaml` 是设置的值。程序里用"填表格"的方式接收
这些设置，第 5 章会看到。

> 小知识：`~` 表示你的主目录（Windows 上是 `C:\Users\你的用户名`，Linux 上是
> `/home/你的用户名`，Mac 上是 `/Users/你的用户名`）。本项目在 Windows 地面站上
> 统一用 `D:\yolo` 作为 YOLO 工作目录，不放在主目录里。

### 0.12 文字和数字

- `'文字'` 或 `"文字"` ＝ **文字**（Python 里叫字符串），`print('你好')` 显示在屏幕上；
- `0`、`15`、`3.5` ＝ **数字**。

文字和数字不能直接拼，要用**填空格**：

```python
print('共 %d 张' % 12)      # %d 填数字 → 输出：共 12 张
print('%s_%06d.jpg' % ('video1', 3))   # %s 填文字；%06d 数字补成 6 位
# 输出：video1_000003.jpg
```

好，够用了。下面开始正题。

---

## 1. 先搞懂四个词

### 1.1 类别名（names）和编号（id）—— 点名册

模型不认识文字，只认数字。`data.yaml` 里的 `names` 是**点名册**：

```yaml
names:
  0: worm          # 编号 0 是青虫（要抓的目标，目前只有这一类）
```

- 标注文件里写 `0` ＝ 青虫，**数字必须和点名册一致**；
- 推理时 `results.names[编号]` 把数字翻译回名字；
- 部署时 `--classes worm` 也是按名字点名——名字对不上就检不出来。

**大白话**：类别名只是给数字起的标签。换名字不用重训，但给已有编号换**含义**
（数字代表的东西变了）必须重训或改标注。以后想加"坏豆荚"等新类别，就往这里
多写一行 `1: damaged_pod`，再用现有 `best.pt` 当起点、换新 `data.yaml` 重新训练
（Ultralytics 会自动适配类别数，不能直接 `resume`）。

### 1.2 置信度（conf）—— 把握程度

置信度是模型对"这个框里确实是这个类别"的**把握程度**（0~1）：

```text
conf = 0.93  →  很确定是青虫
conf = 0.40  →  有点怀疑（可能是叶子卷曲、影子）
```

"确认"就是设一条**及格线**（默认 0.45）：过了及格线才输出，没过就丢掉。

| 现象 | 原因 | 做法 |
|------|------|------|
| 把叶子/影子当虫（误检多） | 及格线太低 | 调高，如 0.6~0.7 |
| 真虫在眼前没检到（漏检多） | 及格线太高/样本不够 | 调低，如 0.3；同时补数据 |

**大白话**：及格线是"部署时的设置"，不改变模型本身。线划得高就少误报多漏报，
划得低就反过来。

### 1.3 边界框（xyxy）—— 画出来的方框

每个检测结果带一个方框，用四个坐标表示：

```text
(x1, y1) ┌────────────┐
         │            │
         └────────────┘ (x2, y2)
```

- `xyxy`：左上角 `(x1, y1)` 和右下角 `(x2, y2)`；
- 由它算出中心点 `center` 和面积 `area`：机器人"左转/右转/前进"看中心点，
  "够近可抓"看面积。

### 1.4 成绩（指标）

训练完看"成绩单"：

| 成绩名 | 大白话 | 对应问题 |
|--------|--------|----------|
| IoU | 预测框和真实框重叠的比例 | 框得准不准 |
| Precision 精确率 | 检出来的里面，有多少是真的 | 误检（把叶子当虫） |
| Recall 召回率 | 真的目标里面，检出多少 | 漏检（虫没框出来） |
| mAP50 | 综合分（重叠一半以上算对） | 整体水平 |
| mAP50-95 | 更严格的综合分 | 框得又准又稳 |

**起步标准**：mAP50 ≥ 0.80，Precision/Recall ≥ 0.85，再配合现场试跑。

---

## 2. 训练全流程（一张图）

```text
拍照/拍视频 → 抽帧 → 整理 → 标注 → 划分数据集 → 写 data.yaml
    → 训练（用现成模型继续学） → 验证（考试） → 部署（让模型看图）
```

每个环节都有代码，复制到 `D:\yolo\scripts\` 文件夹里使用。

---

## 3. 数据准备

### 3.1 目录结构（必须严格）

```text
D:\yolo\datasets\pod_pest\
├── data.yaml
├── images/
│   ├── train/    # 训练图片
│   ├── val/      # 验证图片
│   └── test/     # 测试图片
└── labels/
    ├── train/    # 训练标注（和图片同名）
    ├── val/
    └── test/
```

规则：`images/train/xxx.jpg` 和 `labels/train/xxx.txt` **同名同目录**。

**大白话**：文件夹就是抽屉，必须分好类，程序才找得到。

> 记忆比喻：**train 是平时做作业，val 是月考，test 是期末考**。作业题不能
> 提前给月考抄，所以同一段视频的帧必须分到同一堆（见 3.4）。"数据集"就是
> "图片 + 对应的标注文件"放在一起的总称。

### 3.2 视频抽帧脚本 extract_frames.py

**这个文件干嘛的**：把一段视频变成一张张图片。模型要的是单张照片，而视频是
连续的，所以每隔 15 帧取一张存下来。

**整体思路**（就 3 步）：

1. 打开视频；
2. 一帧一帧读，每隔 15 帧存一张；
3. 读完了关掉视频，打印一共存了几张。

完整代码：

```python
#!/usr/bin/python3
# coding=utf8
"""视频抽帧：把无人机/手机拍的视频变成训练图片。"""
import os
import cv2


def extract_frames(video_path, out_dir='raw_frames', step=15):
    os.makedirs(out_dir, exist_ok=True)
    cap = cv2.VideoCapture(video_path)
    n, saved = 0, 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if n % step == 0:
            base = os.path.splitext(os.path.basename(video_path))[0]
            name = os.path.join(out_dir, '%s_%06d.jpg' % (base, n))
            cv2.imwrite(name, frame)
            saved += 1
        n += 1
    cap.release()
    print('视频 %s：共 %d 帧，抽到 %d 张' % (video_path, n, saved))


if __name__ == '__main__':
    import sys
    for v in sys.argv[1:]:
        extract_frames(v)
```

**分段讲**：

第 1 段——开头三行和工具定义：

```python
#!/usr/bin/python3
# coding=utf8
import os
import cv2

def extract_frames(video_path, out_dir='raw_frames', step=15):
```

`#!/usr/bin/python3` 是给 Linux/Mac 看的"签名"，Windows 会当普通注释跳过，**留着
就行，不用删**；`# coding=utf8` 声明中文不乱码；`import` 打开两个工具箱；`def`
造一个叫 extract_frames 的工具，
带 3 个旋钮：视频路径（必填）、输出文件夹（默认 raw_frames）、隔几帧取一张
（默认 15）。

第 2 段——循环取帧：

```python
while True:
    ok, frame = cap.read()
    if not ok:
        break
```

反复做：读一帧。`cap.read()` 一次给两个东西——`ok`（读成功了吗）和 `frame`
（这帧图片）。读不到（视频放完了）就 `break` 喊停。

第 3 段——每隔 15 帧存一张：

```python
if n % step == 0:
    base = os.path.splitext(os.path.basename(video_path))[0]
    name = os.path.join(out_dir, '%s_%06d.jpg' % (base, n))
    cv2.imwrite(name, frame)
    saved += 1
n += 1
```

`n % step == 0` 意思是"n 是 step 的倍数"，所以每隔 15 帧才进来一次。进来的
时候：从视频路径里取出"不带后缀的文件名"（`base`），拼出图片名
（`video1_000015.jpg` 这种格式），存盘，计数器加 1。无论进没进来，帧号 `n`
每次都要加 1。

第 4 段——收尾和入口：

```python
cap.release()
print('视频 %s：共 %d 帧，抽到 %d 张' % (video_path, n, saved))

if __name__ == '__main__':
    import sys
    for v in sys.argv[1:]:
        extract_frames(v)
```

`cap.release()` 是"用完关掉视频"；`print` 打印结果。最后两行：`if __name__` 
＝"只有直接运行这个文件才执行"；`sys.argv[1:]` ＝ 命令行里第 1 个往后的所有
参数（比如你给的视频文件名们），`for v in ...` 把它们一个一个拿出来，
每次用 extract_frames 工具处理一个。

运行：

```bash
python extract_frames.py 视频1.mp4 视频2.mp4
```

### 3.3 标注格式（LabelImg 导出）

用 [LabelImg](https://github.com/HumanSignal/labelImg) 画框，导出 YOLO 格式。
每张图对应一个 `.txt`，每行一个目标：

```text
0 0.5234 0.4188 0.1211 0.0875
1 0.7345 0.6221 0.0984 0.0762
```

每行五个数字：`类别编号  中心x  中心y  宽  高`。`0.52` 表示目标中心在图片
横向 52% 的位置，宽 `0.12` 表示占图片宽度的 12%。所有数字都换算成 0~1 的
比例，这样图片不管多大都能用。**这些数字不用手写**——LabelImg 画框后自动生成。

### 3.4 划分数据集脚本 split_dataset.py

**这个文件干嘛的**：把标注好的图片，按"来源"分成三堆（train/val/test），
防止模型"背答案"。

**整体思路**（就 3 步）：

1. 按前缀分组：文件名 `video1_000015.jpg` 的前缀是 `video1`，同一段视频的
   帧前缀相同，归到一组；
2. 把所有组名打乱（洗牌），按比例切成三份名单：测试 10%、验证 20%、剩下
   训练；
3. 一组一组复制：图片和它的同名标注一起，复制到对应文件夹。

完整代码：

```python
#!/usr/bin/python3
# coding=utf8
"""划分数据集：同一来源（前缀相同）进同一个集合，防止数据泄漏。"""
import os
import random
import shutil


def split_dataset(src_images, dst_root, val_ratio=0.2, test_ratio=0.1, seed=42):
    random.seed(seed)
    groups = {}
    for name in os.listdir(src_images):
        if not name.endswith('.jpg'):
            continue
        prefix = name.split('_')[0]
        groups.setdefault(prefix, []).append(name)

    prefixes = list(groups.keys())
    random.shuffle(prefixes)
    n_test = max(1, int(len(prefixes) * test_ratio))
    n_val = max(1, int(len(prefixes) * val_ratio))
    test_pre = set(prefixes[:n_test])
    val_pre = set(prefixes[n_test:n_test + n_val])

    for split in ('train', 'val', 'test'):
        os.makedirs(os.path.join(dst_root, 'images', split), exist_ok=True)
        os.makedirs(os.path.join(dst_root, 'labels', split), exist_ok=True)

    for prefix, names in groups.items():
        split = 'test' if prefix in test_pre else ('val' if prefix in val_pre else 'train')
        for name in names:
            img_src = os.path.join(src_images, name)
            txt_src = img_src[:-4] + '.txt'
            shutil.copy(img_src, os.path.join(dst_root, 'images', split, name))
            if os.path.exists(txt_src):
                shutil.copy(txt_src, os.path.join(dst_root, 'labels', split, name[:-4] + '.txt'))

    print('划分完成 → train:%d 组 / val:%d 组 / test:%d 组'
          % (len(prefixes) - n_test - n_val, n_val, n_test))


if __name__ == '__main__':
    import sys
    split_dataset(sys.argv[1], sys.argv[2])
```

**分段讲**：

第 1 段——按前缀分组：

```python
groups = {}
for name in os.listdir(src_images):
    if not name.endswith('.jpg'):
        continue
    prefix = name.split('_')[0]
    groups.setdefault(prefix, []).append(name)
```

把图片文件夹里的文件名一个一个拿出来。`endswith('.jpg')` 问"是不是以 .jpg
结尾"，不是就 `continue`（跳过，看下一个）。是的话，用 `split('_')` 按
下划线切开取第一段当前缀，然后放进 `groups` 这个柜子里——同一前缀的图片
都进同一个格子。`setdefault(prefix, [])` 的意思是"这个格子不存在就先开一个
空的"。

第 2 段——洗牌 + 切三份名单（这段稍微难，拆开讲）：

```python
prefixes = list(groups.keys())
random.shuffle(prefixes)
n_test = max(1, int(len(prefixes) * test_ratio))
n_val = max(1, int(len(prefixes) * val_ratio))
test_pre = set(prefixes[:n_test])
val_pre = set(prefixes[n_test:n_test + n_val])
```

整段意思：把柜子里所有的组名拿出来（`groups.keys()`），放进 `prefixes` 这排
格子，然后**打乱顺序**（`random.shuffle`，像洗牌）。接着按比例数数：总数乘
10% 就是测试组数，乘 20% 就是验证组数。最后从打乱后的格子开头数出前几组当
`test_pre`（测试名单），再往后数几组当 `val_pre`（验证名单），剩下的自动是
训练。

难懂的三小处：

- `int(...)`：把小数去掉变成整数（组数不能是 1.5 组）；`max(1, ...)`：保证
  至少 1 组，万一数据太少也不会分到 0 组；
- `prefixes[:n_test]`：从格子开头数 n_test 个；
  `prefixes[n_test:n_test + n_val]`：从第 n_test 个往后，再数 n_val 个；
- `set(...)`：把切出来的名单装进"一袋子名字"，后面用 `in` 查"在不在袋子
  里"会很快。

第 3 段——建文件夹 + 整组复制：

```python
for split in ('train', 'val', 'test'):
    os.makedirs(os.path.join(dst_root, 'images', split), exist_ok=True)
    os.makedirs(os.path.join(dst_root, 'labels', split), exist_ok=True)

for prefix, names in groups.items():
    split = 'test' if prefix in test_pre else ('val' if prefix in val_pre else 'train')
    for name in names:
        img_src = os.path.join(src_images, name)
        txt_src = img_src[:-4] + '.txt'
        shutil.copy(img_src, os.path.join(dst_root, 'images', split, name))
        if os.path.exists(txt_src):
            shutil.copy(txt_src, os.path.join(dst_root, 'labels', split, name[:-4] + '.txt'))
```

先创建六个文件夹（三堆图片 + 三堆标注，`exist_ok=True` 表示"已存在也别报错"）。
然后一组一组处理：`in` 是问"这个组名在不在某份名单里"，在测试名单就进 test，
在验证名单就进 val，否则进 train。最后把组里的每张图片和它的同名标注（把
`.jpg` 换成 `.txt`）复制过去。`img_src[:-4]` 就是"去掉最后 4 个字符"。

运行：

```bash
python split_dataset.py raw_frames datasets/pod_pest
```

---

## 4. data.yaml 配置单

**这个文件干嘛的**：给程序一张"配置单"，告诉它数据在哪、有哪几类目标。

```yaml
path: D:/yolo/datasets/pod_pest
train: images/train
val: images/val
test: images/test

names:
  0: worm
```

**大白话**：`path` 是数据集根文件夹（改成你的实际路径）；`train/val/test`
分别指三堆图片文件夹；`names` 是**点名册**——顺序就是编号！标注文件里的
数字、训练配置、部署时的 `--classes`，三处必须对得上。

> Windows 注意：`path` 用正斜杠写 `D:/yolo/...`，不要写 `D:\yolo`——反斜杠
> 在 YAML 里是转义符，会出错。
>
> 单类抓虫注意：只识别虫时，除了有虫的图，还要放一批**没有虫**的画面（叶子、
> 豆荚、影子、卷叶）当负样本，**不画框**，模型才不容易把叶子误认成虫。

---

## 5. 训练

### 5.1 训练脚本 train.py

**这个文件干嘛的**：收命令行设置 → 加载别人练好的模型 → 开始训练 → 告诉
你模型存在哪。

**整体思路**（就 4 步）：

1. 用"填表格工具"接收命令行设置；
2. 加载现成模型（预训练权重）；
3. 开始训练；
4. 打印模型保存位置。

完整代码：

```python
#!/usr/bin/python3
# coding=utf8
"""YOLO 训练脚本：用现成模型继续学（迁移学习）。"""
import argparse
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description='训练 YOLO 检测模型')
    parser.add_argument('--data', default='datasets/pod_pest/data.yaml',
                        help='数据集配置')
    parser.add_argument('--model', default='yolov8n.pt',
                        help='预训练权重（现成模型）')
    parser.add_argument('--epochs', type=int, default=100, help='训练轮数')
    parser.add_argument('--batch', type=int, default=16, help='每批图片数')
    parser.add_argument('--imgsz', type=int, default=640, help='输入分辨率')
    parser.add_argument('--device', default='0', help='GPU 用 0，CPU 用 cpu')
    parser.add_argument('--name', default='pod_pest_v1', help='本次训练的输出目录名')
    args = parser.parse_args()

    model = YOLO(args.model)

    model.train(
        data=args.data,
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        device=args.device,
        patience=20,
        project='runs',
        name=args.name,
        verbose=True,
    )

    print('\n训练完成！模型在：')
    print('  runs/%s/weights/best.pt  (部署用这个)' % args.name)
    print('  runs/%s/weights/last.pt  (续训用)' % args.name)


if __name__ == '__main__':
    main()
```

**分段讲**：

第 1 段——打开工具箱 + 填表格：

```python
import argparse
from ultralytics import YOLO

parser = argparse.ArgumentParser(description='训练 YOLO 检测模型')
parser.add_argument('--data', default='...', help='数据集配置')
parser.add_argument('--epochs', type=int, default=100, help='训练轮数')
...
args = parser.parse_args()
```

`argparse` 是"填表格工具"：`ArgumentParser(...)` 造一张空白表格，
`add_argument` 在表格上加一栏（`--data`、`--model`、`--epochs`……每一栏都有
默认值和说明），`parse_args()` 是"收表格"——把命令行里填的内容读进 `args`
盒子。之后 `args.data` 就是取 `--data` 那一栏填的值。`from ultralytics import
YOLO` 意思是"从 ultralytics 工具箱里只要 YOLO 这一个工具"。

第 2 段——加载现成模型：

```python
model = YOLO(args.model)
```

把表格里 `--model` 填的模型文件（默认 `yolov8n.pt`）加载进来。这个文件是
别人已经用超大公开数据集练好的**现成模型**——已经会看边缘、纹理、形状这些
通用特征。我们拿它当起点，只教它认我们的青虫，这叫"用现成模型继续学"
（迁移学习）。这就是为什么几百张图就能训出可用模型。

第 3 段——开始训练 + 打印结果：

```python
model.train(
    data=args.data,
    epochs=args.epochs,
    ...
    verbose=True,
)
print('  runs/%s/weights/best.pt  (部署用这个)' % args.name)
```

`train` 是"开始训练"工具，后面括号里的每一项都是旋钮：用哪套数据、看几遍、
一次看几张、图片缩放到多大、用哪块显卡。`verbose=True` 让训练过程打印到
屏幕。训练完用 `%s`（填空格）把输出目录名填进提示文字打印出来。

运行：

```bash
cd D:\yolo
python scripts/train.py \
  --data datasets/pod_pest/data.yaml \
  --model yolov8n.pt \
  --epochs 100 --batch 16 --imgsz 640 --device 0
```

命令里的 `\` 只是"这一行没写完，下一行继续"的意思，可以写成一行。

> 两个词提前说清楚：
>
> - **预训练权重**（`yolov8n.pt`）：别人练好的"现成模型文件"；
> - **迁移学习**：拿现成模型当起点，只教它认我们的目标。相当于"有基本功的
>   实习生补一节专业课"，而不是从婴儿开始教。

### 5.2 命令行等价写法

不想写脚本时，用 ultralytics 自带的命令行工具：

```bash
yolo detect train \
  data=datasets/pod_pest/data.yaml \
  model=yolov8n.pt epochs=100 imgsz=640 batch=16 device=0 \
  patience=20 project=runs name=pod_pest_v1
```

效果和脚本一样，只是设置写在命令里（`名字=值` 的格式，而不是 `--名字 值`）。
新手建议先用 5.1 的脚本方式，出错信息更清楚。

### 5.3 关键旋钮解释

| 旋钮 | 干什么的 | 建议 |
|------|----------|------|
| `epochs` 训练轮数 | 把训练集完整看多少遍 | 先 100，看曲线再调 |
| `batch` 每批图片数 | 一次同时看几张 | 8G 显卡：n=32、s=16、m=8；不够就减半 |
| `imgsz` 图片大小 | 图片缩放到多大再喂给模型 | 640 起步；部署到树莓派再降到 480/320 |
| `device` | 用哪块显卡 | 地面站必须 0 |
| `patience` | 连续多少轮没进步就提前停 | 20 左右 |
| `lr0` | 每次"学多少"的步子大小 | 默认即可，异常再调 |

### 5.4 训练完看什么

打开 `runs/pod_pest_v1/`：

| 文件 | 是什么 | 怎么看 |
|------|--------|--------|
| `weights/best.pt` | 成绩最好的模型 | **部署用这个** |
| `weights/last.pt` | 最后一轮的模型 | 续训用 |
| `results.png` | 成绩曲线图 | 看曲线是否平稳上升 |
| `confusion_matrix.png` | 认错表 | 看有没有把叶子/豆荚误认成虫 |
| `results.csv` | 每一轮的数字记录 | 导入表格软件分析 |
| `args.yaml` | 本次训练的所有设置 | 复盘用 |

看曲线：右边 `Precision`、`Recall`、`mAP50(B)` 是否上升后变平。如果训练时
成绩一直涨、验证时一直不涨，就是"背题背过头"（过拟合）：数据太少或标注乱。

### 5.5 中断了怎么办

```bash
yolo detect train model=runs/pod_pest_v1/weights/last.pt resume=True
```

`last.pt` 不光存了模型，还记了进度；`resume=True` ＝ 接着上次继续，不用从头跑。

---

## 6. 验证与"确认"

### 6.1 考试命令

```bash
yolo detect val \
  model=runs/pod_pest_v1/weights/best.pt \
  data=datasets/pod_pest/data.yaml
```

整句大白话：**用 yolo 工具给模型考试**——`model=` 考哪个模型，`data=` 用哪套
题（data.yaml 里的 val 月考卷）。加 `split=test` 就是换成期末卷（第一次见的
数据，最接近真实水平）：

```bash
yolo detect val \
  model=runs/pod_pest_v1/weights/best.pt \
  data=datasets/pod_pest/data.yaml split=test
```

### 6.2 用 Python 拿成绩

```python
from ultralytics import YOLO

model = YOLO('runs/pod_pest_v1/weights/best.pt')
metrics = model.val(data='datasets/pod_pest/data.yaml')

print('mAP50   :', round(metrics.box.map50, 3))
print('mAP50-95:', round(metrics.box.map, 3))
print('Precision:', round(metrics.box.mp, 3))
print('Recall   :', round(metrics.box.mr, 3))
```

整体意思：加载模型 → 用 val 工具考试 → 把"成绩单"放进 `metrics` 盒子 →
打印几项成绩。`metrics.box.map50` 就是"从成绩单里取综合分"；`round(..., 3)`
把数字保留到小数点后 3 位，看起来清爽。

### 6.3 推理/确认脚本 predict.py

**这个文件干嘛的**：训练完后实际用模型——图片、视频、摄像头都能喂给它，
它框出目标，连续 3 帧都确认才打印"确认"。

**整体思路**（就 4 步）：

1. 定三个"筛选规则"：及格线、只关心哪些类别、最小面积；
2. 写一个"筛框工具"：把模型给的一堆框过三关；
3. 主循环：读一帧 → 检测 → 筛框 → 画框 → 显示；
4. 连续 3 帧都确认才打印，按 ESC 退出。

完整代码：

```python
#!/usr/bin/python3
# coding=utf8
"""推理/确认脚本：图片、视频、摄像头都支持。"""
import cv2
from ultralytics import YOLO

CONF = 0.45
TARGET_CLASSES = {'worm'}
MIN_AREA = 100


def confirm_targets(results):
    found = []
    names = results.names
    for box in results.boxes:
        cls_id = int(box.cls[0])
        name = names[cls_id]
        conf = float(box.conf[0])
        x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
        area = (x2 - x1) * (y2 - y1)

        if name not in TARGET_CLASSES:
            continue
        if conf < CONF:
            continue
        if area < MIN_AREA:
            continue
        found.append({
            'class': name,
            'conf': conf,
            'center': ((x1 + x2) // 2, (y1 + y2) // 2),
            'area': area,
        })
    return found


def main():
    model = YOLO('runs/pod_pest_v1/weights/best.pt')
    source = input('输入图片/视频路径，或 0 用摄像头: ') or '0'
    cap = cv2.VideoCapture(0) if source == '0' else cv2.VideoCapture(source)

    confirm_streak = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        results = model.predict(frame, conf=CONF, verbose=False, device=0)[0]
        found = confirm_targets(results)
        frame = results.plot()

        if found:
            confirm_streak += 1
            if confirm_streak >= 3:
                print('[确认] 青虫 conf=%.2f center=%s' % (found[0]['conf'], found[0]['center']))
        else:
            confirm_streak = 0

        cv2.imshow('predict', frame)
        if cv2.waitKey(1) == 27:
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
```

**分段讲**：

第 1 段——三个筛选规则：

```python
CONF = 0.45
TARGET_CLASSES = {'worm'}
MIN_AREA = 100
```

三行三个设置：及格线 0.45（想少误报就调高）；只关心 `worm` 这个类别——花括号
是"一袋子名字"，后面用 `in` 检查在不在袋子里；框面积小于 100 的丢掉（太小
多半是远处误检）。

第 2 段——筛框工具：

```python
def confirm_targets(results):
    found = []
    names = results.names
    for box in results.boxes:
        cls_id = int(box.cls[0])
        name = names[cls_id]
        conf = float(box.conf[0])
        x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
        area = (x2 - x1) * (y2 - y1)
        if name not in TARGET_CLASSES:
            continue
        if conf < CONF:
            continue
        if area < MIN_AREA:
            continue
        found.append({
            'class': name,
            'conf': conf,
            'center': ((x1 + x2) // 2, (y1 + y2) // 2),
            'area': area,
        })
    return found
```

整体意思：模型一次给出一堆框（`results.boxes`），这个工具把每个框拿出来，
过三关——**类别对（在不在点名册的袋子里）、把握够（conf 过及格线）、面积够**。
三关都过的，记成一条"带名字的柜子"（类别、把握、中心点、面积），放进 `found`
这排格子，最后整排交回去（`return`）。

难懂的小地方：

- `box.cls[0]`：这个框的类别编号；`results.names[编号]` 查点名册得到名字；
- `int(...)` / `float(...)`：把值转成整数/小数，方便比较；
- `[int(v) for v in box.xyxy[0]]`：把框的四个角坐标都转成整数——这行是
  "把一排格子里的每个数都转一遍"的简写；
- `((x1 + x2) // 2, ...)`：算框中心点，`//` 是整除（结果不带小数）。

第 3 段——主循环：

```python
model = YOLO('runs/pod_pest_v1/weights/best.pt')
source = input('输入图片/视频路径，或 0 用摄像头: ') or '0'
cap = cv2.VideoCapture(0) if source == '0' else cv2.VideoCapture(source)

confirm_streak = 0
while True:
    ok, frame = cap.read()
    if not ok:
        break
    results = model.predict(frame, conf=CONF, verbose=False, device=0)[0]
    found = confirm_targets(results)
    frame = results.plot()
    if found:
        confirm_streak += 1
        if confirm_streak >= 3:
            print('[确认] 青虫 conf=%.2f center=%s' % (found[0]['conf'], found[0]['center']))
    else:
        confirm_streak = 0
    cv2.imshow('predict', frame)
    if cv2.waitKey(1) == 27:
        break
cap.release()
cv2.destroyAllWindows()
```

整体意思：先加载模型；然后 `input()` 在终端问你要喂什么（直接回车＝用
摄像头 0）。进主循环：读一帧 → 检测（`verbose=False` 是"别打印一堆过程"，
`[0]` 是"只要第一个结果"）→ 用上面的筛框工具筛 → 把框画到画面上 →
弹窗显示。`confirm_streak` 是"连续确认计数器"：这一帧有目标就加 1，连续
3 帧都有才打印"确认"（单帧可能是噪声）；这一帧没有就清零。`cv2.waitKey(1)`
是"停一下等键盘"，`27` 是 ESC 键的编号，按 ESC 退出。最后关视频、关窗口。

运行：

```bash
python predict.py --model runs/pod_pest_v1/weights/best.pt --source 0
```

---

## 7. 常见问题

| 现象 | 可能原因 | 解决 |
|------|----------|------|
| 显卡内存不够（`CUDA out of memory`） | 一次看的图太多 | `batch` 减半；换小模型；图片缩小 |
| 训练很快但综合分 = 0 | 标注编号和点名册对不上 | 抽查 txt 第一列数字；核对 data.yaml 顺序 |
| 训练成绩涨、验证不涨 | 背题背过头（过拟合） | 数据太少/标注乱；加数据；提前停 |
| 某类成绩特别低 | 该类样本太少/太像别的类 | 补该类样本；加负样本 |
| 验证比训练好很多 | 背答案（同源帧跨集合） | 用 3.4 节脚本重分 |
| 现场效果比测试差 | 光照/角度和训练数据差太远 | 用现场视角补拍 |
| 误检多 | 及格线低/负样本少 | 调高 `CONF`；加没有目标的画面 |
| 漏检多 | 及格线高/小目标/样本少 | 调低 `CONF`；图片放大；补小目标样本 |
| 部署端检不出来 | 类别名不一致 | 打印 `results.names`，和 data.yaml 核对 |
| `IndentationError` | 缩进（空格）不对 | 检查行首空格，和教程保持一致 |
| `ModuleNotFoundError` | 缺工具箱 | 装对应的库：`pip3 install 工具箱名` |

---

## 8. 动手练习

1. 用 20 张图、5 轮跑一次 mini 训练，打开 `results.png`，说出横纵坐标是什么；
2. 把 `CONF` 从 0.45 改成 0.8 再推理，记录误检/漏检变化；
3. 把 `TARGET_CLASSES` 改成一个不存在的名字（如 `bug`）再推理，观察输出变成空；
4. 打印每个框的 `cls / conf / xyxy`，和画面手工核对；
5. 用 3.4 节脚本划分数据集，检查三堆里没有同前缀文件混集；
6. 换 `yolov8s` 训练对比时间和成绩，体会模型大小的影响；
7. （新手必做）故意删掉一个冒号或改乱缩进跑一次，学会读报错。

---

## 附录 A：命令速查

| 用途 | 命令 |
|------|------|
| 抽帧 | `python scripts/extract_frames.py 视频.mp4` |
| 划分数据集 | `python scripts/split_dataset.py raw_frames datasets/pod_pest` |
| 训练 | `python scripts/train.py --data datasets/pod_pest/data.yaml --model yolov8n.pt` |
| 续训 | `yolo detect train model=runs/xxx/weights/last.pt resume=True` |
| 验证 | `yolo detect val model=runs/xxx/weights/best.pt data=datasets/pod_pest/data.yaml` |
| 测试集评估 | 同上加 `split=test` |
| 推理 | `python scripts/predict.py` |
| 导出通用格式 | `yolo export model=runs/xxx/weights/best.pt format=onnx imgsz=640` |

## 附录 B：黑话对照表（遇到看不懂先查这里）

| 黑话 | 大白话 |
|------|--------|
| 库 / 模块 | 工具箱 |
| 函数 | 工具 |
| 参数 | 旋钮 / 设置 |
| 变量 | 贴了名字的盒子 |
| 调用函数 | 使用工具 |
| 返回值 | 工具交回来的结果 |
| 注释 | 便利贴（`#`，电脑跳过） |
| 列表 | 一排有顺序的格子 |
| 字典 | 带名字标签的柜子 |
| 集合 | 一袋子名字 |
| 循环 | 反复做 |
| 判断 if | 岔路口 |
| 缩进 | 用空格分组（Python 的规矩） |
| 字符串 | 文字 |
| 整数 | 不带小数的数字 |
| 布尔 True/False | 是 / 否 |
| 切片 `[:]` | 从格子里切一段 |
| 编码 | 文字的翻译表（防乱码） |
| 置信度 conf | 把握程度 |
| 阈值 | 及格线 |
| 推理 | 让模型看图找目标 |
| 训练 | 教模型 |
| 验证 / 测试 | 考试 |
| 指标 / metrics | 成绩 |
| 数据集 | 图片 + 标注文件放一起 |
| 标注 | 给图片里的目标画框并写类别 |
| 归一化 | 换算成 0~1 的比例 |
| 预训练权重 | 别人练好的现成模型文件 |
| 迁移学习 | 拿现成模型当起点，继续教它认我们的目标 |
| 过拟合 | 背题背过头，只会背不会举一反三 |
| 数据泄漏 | 模型背答案（同源数据混进考试卷） |
| 断点续训 | 接着上次的进度继续练 |
| 混淆矩阵 | 认错表（哪两类容易互相认错） |
| mAP / mAP50 | 综合分 |
| IoU | 两个框重叠的比例 |
| epochs | 把训练集完整看几遍 |
| batch | 一次同时看几张图 |
| imgsz | 图片缩放到多大 |
| GPU / CUDA | 显卡 / 用显卡加速 |
