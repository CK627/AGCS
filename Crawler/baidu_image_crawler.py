#!/usr/bin/env python3
# coding=utf8
"""百度图片爬虫：按关键词爬取图片，用于 YOLO 训练数据采集。

纯标准库实现（urllib + cookie jar），无需额外安装依赖。

用法：
    python3 baidu_image_crawler.py --keyword 大青虫 --num 50
    python3 baidu_image_crawler.py --keyword 大青虫 菜青虫 --num 30

保存位置：data/<关键词>/xxx.jpg（默认），清洗后移入训练数据集。
"""
import argparse
import http.cookiejar
import json
import os
import ssl
import time
import urllib.error
import urllib.parse
import urllib.request

UA = ('Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) '
      'AppleWebKit/537.36 (KHTML, like Gecko) '
      'Chrome/120.0.0.0 Safari/537.36')

# 搜索接口用的请求头（必须带 X-Requested-With，否则百度返回 antiFlag 风控）
API_HEADERS = {
    'User-Agent': UA,
    'Referer': 'https://image.baidu.com/',
    'Accept': 'application/json, text/javascript, */*; q=0.01',
    'X-Requested-With': 'XMLHttpRequest',
    'Accept-Language': 'zh-CN,zh;q=0.9',
}

# 下载图片用的请求头
IMG_HEADERS = {
    'User-Agent': UA,
    'Referer': 'https://image.baidu.com/',
    'Accept': 'image/webp,image/apng,image/*,*/*;q=0.8',
}

# ---------------- 百度图片 URL 解密 ----------------
# 接口返回的 objURL 可能是混淆串（ippr=http、ipprf=https），需要两步还原：
# 1) 替换多字符标记；2) 按字符映射表翻译单字符
TOKEN_TABLE = {'_z2C$q': ':', '_z&e3B': '.', 'AzdH3F': '/'}
CHAR_TABLE = {
    'w': 'a', 'k': 'b', 'v': 'c', '1': 'd', 'j': 'e', 'u': 'f', '2': 'g',
    'i': 'h', 't': 'i', '3': 'j', 'h': 'k', 's': 'l', '4': 'm', 'g': 'n',
    '5': 'o', 'r': 'p', 'q': 'q', '6': 'r', 'f': 's', 'p': 't', '7': 'u',
    'e': 'v', 'o': 'w', '8': '1', 'd': '2', 'n': '3', '9': '4', 'c': '5',
    'm': '6', '0': '7', 'b': '8', 'l': '9', 'a': '0',
}


def decode_url(url):
    """把百度混淆图片地址还原成真实 URL；普通 URL 原样返回。"""
    if not url or ('z2C$q' not in url and not url.startswith('ippr')):
        return url
    for k, v in TOKEN_TABLE.items():
        url = url.replace(k, v)
    return url.translate(str.maketrans(CHAR_TABLE))


def make_opener():
    """创建带 cookie 的 opener。

    百度接口需要先访问搜索页拿到 BAIDUID cookie，否则返回
    "Forbid spider access"。macOS 自带 Python 常缺系统证书链，
    图片采集场景直接使用宽松 SSL 上下文。
    """
    ctx = ssl._create_unverified_context()
    cj = http.cookiejar.CookieJar()
    opener = urllib.request.build_opener(
        urllib.request.HTTPCookieProcessor(cj),
        urllib.request.HTTPSHandler(context=ctx),
    )
    return opener


def seed_cookies(opener, keyword):
    """访问一次搜索页种下 cookie，返回搜索页 URL（给接口当 Referer）。"""
    search_url = ('https://image.baidu.com/search/index?tn=baiduimage&word='
                  + urllib.parse.quote(keyword))
    try:
        headers = dict(API_HEADERS)
        headers['Referer'] = search_url
        req = urllib.request.Request(search_url, headers=headers)
        opener.open(req, timeout=15).read()
    except Exception:
        pass
    return search_url


def http_get(opener, url, referer=None, headers=None):
    """GET 请求，返回 (bytes, headers)。"""
    headers = dict(headers or IMG_HEADERS)
    if referer:
        headers['Referer'] = referer
    req = urllib.request.Request(url, headers=headers)
    with opener.open(req, timeout=8) as resp:
        return resp.read(), resp.headers


def fetch_image_urls(opener, referer, keyword, page, per_page=30):
    """请求百度图片搜索接口，返回该页的图片 URL 列表。"""
    pn = page * per_page
    params = {
        'tn': 'resultjson_com',
        'ipn': 'rj',
        'word': keyword,
        'pn': pn,
        'rn': per_page,
        'gsm': hex(pn + per_page)[2:],   # 百度接口校验值（pn+rn 的十六进制）
        'ie': 'utf-8',
        'oe': 'utf-8',
        'width': '',
        'height': '',
    }
    url = 'https://image.baidu.com/search/acjson?' + urllib.parse.urlencode(params)
    # 百度风控是概率性的：失败就稍等重试，最多 3 次
    data = None
    for attempt in range(3):
        try:
            body, _ = http_get(opener, url, referer=referer, headers=API_HEADERS)
            obj = json.loads(body.decode('utf-8', 'ignore'))
        except Exception as e:
            print('  请求搜索接口失败（第 %d 次）: %s' % (attempt + 1, e))
            data = None
        else:
            # 风控时返回 {"antiFlag": 1, "message": "Forbid spider access"}
            if isinstance(obj, dict) and obj.get('antiFlag'):
                print('  被风控（antiFlag），%d 秒后重试...' % (3 * (attempt + 1)))
                data = None
            else:
                data = obj.get('data', [])
        if isinstance(data, list):
            break
        time.sleep(3 * (attempt + 1))

    if not isinstance(data, list):
        return []

    urls = []
    for item in data:
        if not isinstance(item, dict):
            continue
        # 下载优先级：百度 CDN（thumb/middle/hover）最稳 → 原站 objURL 兜底
        img_url = (item.get('thumbURL') or item.get('middleURL')
                   or item.get('hoverURL') or item.get('objURL'))
        if img_url:
            u = decode_url(urllib.parse.unquote(img_url))
            if u.startswith('http'):
                urls.append(u)
    return urls


def image_ext(content):
    """按文件头识别图片格式，返回扩展名；不是图片返回 None。"""
    if content[:3] == b'\xff\xd8\xff':
        return 'jpg'                             # JPEG
    if content[:4] == b'\x89PNG':
        return 'png'                             # PNG
    if content[:3] == b'GIF':
        return 'gif'                             # GIF
    if content[:2] == b'BM':
        return 'bmp'                             # BMP
    if content[:4] == b'RIFF' and content[8:12] == b'WEBP':
        return 'webp'                            # WebP（百度图库常用，别漏掉）
    return None


def download_image(opener, url, path_base, referer=None):
    """下载单张图片，按真实格式保存为 path_base.<扩展名>；失败返回 None。

    referer 传搜索页 URL——百度图库 CDN 按 Referer 防盗链，
    不带/带错 Referer 会握手超时或返回非图片内容。
    """
    try:
        body, headers = http_get(opener, url, referer=referer)
        ctype = headers.get('Content-Type', '')
        ext = image_ext(body)
        # 双重校验：Content-Type 和文件头
        if not (ctype.startswith('image/') or ext):
            return None
        if not ext:
            return None
        path = path_base + '.' + ext
        with open(path, 'wb') as f:
            f.write(body)
        return path
    except Exception:
        return None


def crawl_keyword(opener, referer, keyword, num, out_dir, max_pages=10):
    """按一个关键词爬 num 张图片到 out_dir/<keyword>/。"""
    save_dir = os.path.join(out_dir, keyword)
    os.makedirs(save_dir, exist_ok=True)

    seen = set()          # 去重
    saved = 0
    page = 0
    print('[爬取] 关键词：%s，目标 %d 张 → %s' % (keyword, num, save_dir))

    while saved < num and page < max_pages:
        urls = fetch_image_urls(opener, referer, keyword, page)
        if not urls:
            print('  第 %d 页没有拿到图片，可能触发了风控，停止' % (page + 1))
            break
        page_ok = 0
        for url in urls:
            if saved >= num:
                break
            if url in seen:
                continue
            seen.add(url)
            path_base = os.path.join(save_dir, '%s_%03d' % (keyword, saved + 1))
            saved_path = download_image(opener, url, path_base, referer=referer)
            if saved_path:
                saved += 1
                page_ok += 1
                print('  已保存 %d/%d：%s' % (saved, num, os.path.basename(saved_path)))
            else:
                print('  下载失败：%s' % url[:80])
            time.sleep(0.3)      # 礼貌间隔，降低被封概率
        if page_ok == 0:
            print('  本页一张都没下载成功，停止（图片服务可能被限流）')
            break
        page += 1
        time.sleep(0.6)

    print('[完成] %s 共保存 %d 张' % (keyword, saved))
    return saved


def main():
    parser = argparse.ArgumentParser(description='百度图片爬虫（YOLO 训练数据采集）')
    parser.add_argument('--keyword', nargs='+', default=['大青虫'],
                        help='关键词，可多个（每个存一个子目录）')
    parser.add_argument('--num', type=int, default=50, help='每个关键词下载张数')
    parser.add_argument('--out', default='data', help='输出根目录（默认 data/）')
    args = parser.parse_args()

    # 先访问搜索页种 cookie（一次即可，多个关键词共用）
    opener = make_opener()
    referer = seed_cookies(opener, args.keyword[0])

    total = 0
    for kw in args.keyword:
        total += crawl_keyword(opener, referer, kw, args.num, args.out)
    print('\n全部完成，共保存 %d 张图片到 %s/' % (total, args.out))


if __name__ == '__main__':
    main()
