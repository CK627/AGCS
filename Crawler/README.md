# Crawler —— 百度图片爬虫（训练数据采集）

按关键词从百度图片爬取图片，用于 YOLO 训练数据的前期采集。

> 说明：`大青虫` 是俗称（百度搜索结果会混入多种绿色幼虫），**爬完必须人工
> 清洗**——删掉错图、水印图、卡通图，只保留真实虫态照片，再进入标注环节。
> 图片仅用于学习/科研数据集，注意版权与肖像权，商用需自行确认授权。

## 用法

```bash
cd /Users/jj/Documents/MyCode/AGCS/Crawler

# 爬"大青虫" 50 张 → data/大青虫/
python3 baidu_image_crawler.py --keyword 大青虫 --num 50

# 多关键词（每个存一个子目录）
python3 baidu_image_crawler.py --keyword 大青虫 菜青虫 豆虫 --num 30
```

纯标准库实现，无需安装依赖。参数：

| 参数 | 默认 | 说明 |
|------|------|------|
| `--keyword` | 大青虫 | 关键词，可多个 |
| `--num` | 50 | 每个关键词下载张数 |
| `--out` | data | 输出根目录 |

## 清洗后怎么进训练集

```bash
# 1. 人工看一遍，删掉错图/水印图/重复图
# 2. 把清洗后的图片收集起来（先别直接塞进 images/train）
mkdir -p ../MacYoLo/datasets/pod_pest/raw_baidu_worm
cp data/大青虫/*.jpg ../MacYoLo/datasets/pod_pest/raw_baidu_worm/
```

> 注意：爬来的图片不能直接当训练集用——需要先清洗，再用 LabelImg 标注
> 青虫位置（类别 `worm`），然后划分 train/val/test、训练。
> 流程见 [GroundStation/YOLO训练教程.md](../GroundStation/YOLO训练教程.md)。

## 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| 一直提示"没有拿到图片" | 百度风控/网络 | 等一会儿再试；换关键词；减少 `--num` |
| 保存的图打不开 | 下载到了非图片内容 | 脚本已按文件头校验，仍可人工删除坏的 |
| 存下来的多是 `.webp` | 百度图库返回 WebP 格式 | 正常现象，脚本按真实格式命名；ultralytics/OpenCV 都能直接读 |
| 接口地址是 `ipprf_z2C$q...` | 百度对图片 URL 做了混淆 | 脚本已内置解密（ippr=http、ipprf=https + 字符映射） |
| 图太少/不相关 | 关键词太宽泛 | 加限定词，如"大青虫 大豆""菜青虫 幼虫" |
