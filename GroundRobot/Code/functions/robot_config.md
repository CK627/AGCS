# robot_config.py 教学文档

## 1. 这个文件是做什么的

`robot_config.py` 只做一件事：读取 `config/robot_params.yaml`，把里面的参数变成
Python 字典，供其他模块调用。

```python
params = load_params()
params['vision']['target_color']   # -> 'red'
params['walk']['stride']           # -> 60
```

**为什么需要配置文件？** 因为"目标颜色、步幅、避障阈值、抓取高度"这类值需要经常
调整。如果硬写在代码里，每次改参数都要改代码、重新同步；写在 YAML 里，改完文件
重启程序即可，而且不熟悉的同学也能安全地调参。

## 2. 新手必读：`params['vision']['target_color']` 是什么意思

> 如果你完全没接触过字典（dict），这一节必须读完。后面所有模块代码里到处都是
> `params['walk']['stride']` 这种写法，原理完全一样。

### 2.1 先搞懂 `load_params()` 是什么：函数（function）

`load_params()` 是**调用一个函数**。函数就是"一段起好了名字、可以反复使用的代码"。

在 `robot_config.py` 文件里，我们**定义**了这个函数：

```python
def load_params(path=_DEFAULT_PATH):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)
```

逐段看：

- `def` 是 "define（定义）" 的缩写，意思是"我要定义一个函数"
- `load_params` 是函数的名字，意思是 "load parameters"（加载参数）
- `(path=_DEFAULT_PATH)` 是参数：调用时不写就使用默认的配置文件路径，
  也可以传入别的路径
- 缩进的几行是**函数体**：打开文件 → 用 `yaml.safe_load` 解析 →
  `return` 把结果交出来

**关键概念：定义 ≠ 执行。**

- 写 `def load_params(...)` 只是告诉 Python"有这么个功能"，此时**不会**读文件
- 只有在代码里写下 `load_params()`（带括号）才是**调用**，才会真正执行函数体
- 函数执行到 `return` 时，会把后面的结果"交还"给调用它的地方

`load_params` 这个名字的含义：**把配置文件里的参数从磁盘读进内存**。
参数在 YAML 文件里只是一段文字，程序要使用它们，必须先变成能计算的
Python 数据——`yaml.safe_load` 就负责这个"文字 → 数据"的转换。

### 2.2 `params = load_params()` 这一行做了什么

```python
params = load_params()
```

这一行可以拆成两步：

1. **先算右边**：`load_params()` 执行函数，返回一个装满参数的字典
2. **再存左边**：`=` 把返回的字典存进变量 `params`，之后想用参数就从
   `params` 里取

这和 `x = 1 + 2` 是同一个道理：先算出 `3`，再存进 `x`。`=` 右边永远先执行。

### 2.3 先看配置文件长什么样

`config/robot_params.yaml` 里是这样写的（**缩进就是层级**）：

```yaml
vision:
  target_color: red        # vision 这一组里，有个叫 target_color 的键，值是 red
  min_area: 500

walk:
  stride: 60               # walk 这一组里，有个叫 stride 的键，值是 60
  speed: 50
```

### 2.4 `load_params()` 把 YAML 变成 Python 字典

Python 的**字典（dict）**就是一堆"名字 → 值"的对应关系，用花括号 `{}` 表示：

```python
{
    'vision': {'target_color': 'red', 'min_area': 500},
    'walk':   {'stride': 60, 'speed': 50},
}
```

注意：YAML 里 `vision:` 下面又缩进了两行，所以变成字典后，`vision` 这个"值"
**本身又是一个字典**（嵌套）。这就是"嵌套字典"。

### 2.5 字典取值 = 打开带标签的柜子

字典用 `变量名[钥匙名]` 取值。把 `params` 想象成一个储物柜：

```
params
 ├── 格子「vision」 ──▶ 里面是个小柜子
 │     ├── 小格「target_color」 ──▶ 'red'
 │     └── 小格「min_area」     ──▶ 500
 └── 格子「walk」   ──▶ 里面是个小柜子
       ├── 小格「stride」 ──▶ 60
       └── 小格「speed」  ──▶ 50
```

所以要取 `target_color`，得先打开外层格子，再打开内层小格，用两个 `[]`：

```python
params['vision']                  # 打开 vision 格子 → {'target_color': 'red', 'min_area': 500}
params['vision']['target_color']  # 再打开 target_color 小格 → 'red'
```

### 2.6 文档开头那两个例子逐行看

```python
params = load_params()
params['vision']['target_color']   # -> 'red'
params['walk']['stride']           # -> 60
```

- `#` 是注释符号，后面的内容只是"这行会得出什么"，不是代码
- `-> 'red'` 表示：`params['vision']['target_color']` 这个表达式的结果是
  字符串 `'red'`（YAML 里没加引号的 red，Python 认作字符串）
- `-> 60` 表示：`params['walk']['stride']` 的结果是整数 `60`（YAML 里的数字
  Python 会转成整数）

### 2.7 常见错误：KeyError

钥匙名写错（大小写、下划线、拼写）会报错：

```python
params['vision']['Target_color']   # KeyError: 'Target_color'
```

不确定有哪些钥匙时，打印出来看：

```python
params = load_params()
print(params)                   # 打印整个字典
print(params['vision'].keys())  # 打印 vision 组里有哪些键
```

### 2.8 为什么非要用这种写法

因为参数放在 YAML 里，代码只负责"按路径取"。想改目标颜色，改 YAML 里的
`target_color: green` 即可，代码不用动。`['vision']['target_color']` 这条
"路径"正好对应 YAML 的缩进层级，一眼能对上。

## 3. 用到了哪些模块

| 模块 | 来源 | 作用 |
|------|------|------|
| `os` | Python 标准库 | 拼接文件路径 |
| `sys` | Python 标准库 | 修改模块搜索路径 `sys.path` |
| `yaml` | 第三方（树莓派镜像预装，`pip3 install pyyaml`） | 解析 YAML 文本为 Python 对象 |

这里没有用到机器人官方 SDK，是纯粹的通用代码。

## 4. 逐段讲解

### 4.1 前 8 行：路径引导（最容易被忽略、也最容易出问题）

```python
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)
```

一步步拆解：

1. `__file__` 是当前文件路径。在树莓派上是
   `/home/pi/spiderpi/functions/robot_config.py`
2. `os.path.abspath(__file__)` 把它变成绝对路径（防止用相对路径启动时出错）
3. `os.path.dirname(...)` 取所在目录，第一次得到
   `/home/pi/spiderpi/functions`，**再套一次**得到 `/home/pi/spiderpi`（项目根）
4. `sys.path` 是 Python 找模块的搜索列表。直接运行
   `python3 robot_config.py` 时，Python 只把脚本所在目录（`functions/`）加入
   搜索路径，此时 `import functions.xxx` 会失败——因为 Python 不知道
   `functions` 这个包在哪
5. 所以把项目根目录插到 `sys.path` 最前面，之后 `from functions.vision_utils
   import ...` 就能找到

> 排查技巧：如果出现 `ModuleNotFoundError: No module named 'functions'`，
> 99% 是这段路径引导没生效（文件被挪了位置，或用了打包工具改变了 `__file__`）。

### 4.2 读取参数

```python
_DEFAULT_PATH = os.path.join(_PKG_ROOT, 'config', 'robot_params.yaml')

def load_params(path=_DEFAULT_PATH):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)
```

- `os.path.join` 用系统正确的分隔符拼接路径，不要手写 `'/'`
- `with open(...) as f:` 会在离开代码块后自动关闭文件，避免文件句柄泄漏
- `yaml.safe_load` 只解析标准 YAML 数据，比 `yaml.load` 安全。官方 SDK 的
  `yaml_handle.py` 用的是 `yaml.load(..., Loader=yaml.FullLoader)`，效果等价
- 返回值是嵌套 dict，与 YAML 的缩进结构一一对应

> 函数的概念（`def` 定义、调用、`return` 返回值）见上面第 2.1 节。

## 5. 关键 API 速查

| 调用 | 说明 |
|------|------|
| `load_params()` | 返回整个参数 dict |
| `load_params(path='/tmp/xx.yaml')` | 读取指定位置的参数文件（调试用） |

## 6. 常见问题排查

| 现象 | 可能原因 | 排查/解决 |
|------|----------|-----------|
| `FileNotFoundError: ... robot_params.yaml` | 配置文件没同步到树莓派 | 检查 `~/spiderpi/config/robot_params.yaml` 是否存在；重新 `./sync_to_robot.sh` |
| `yaml.parser.ParserError` | YAML 缩进或格式错误 | 用编辑器打开看缩进，YAML 不允许混用 Tab 和空格 |
| `ImportError: No module named 'yaml'` | 树莓派没装 pyyaml | `pip3 install pyyaml`（需 STA 局域网模式联网） |
| `KeyError: 'walk'` | 参数文件被改坏/漏了字段 | 用 `python3 robot_config.py` 直接打印，对照 `robot_params.yaml` |

## 7. 动手练习

1. 在 `robot_params.yaml` 里加一个 `speaker.volume: 80`，然后在
   `load_params()` 后打印出来
2. 把 `load_params` 改成也支持 `.json` 文件（提示：`json.load`）
3. 故意把 YAML 缩进写错，观察报错信息，学会读 `ParserError` 提示
