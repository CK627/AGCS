# H3C 无线网络配置（单独手册）

> 本手册只讲 **H3C** 设备。目标：让地面站、SpiderPi Pro、Minihomer 基站连到
> 同一张本地局域网。网络配置属于基础设施，不是最优先的活；请先读懂
> [通信协议规范.md](通信协议规范.md)，再回来按本手册配置。
>
> 适用设备：H3C WX 系列 AC + WA 系列 Fit AP，或 WA 系列 Fat AP。
> 平台：Comware V7/V9。命令以常见版本为例，实际以
> `display version` / `display current-configuration` 为准。

## 1. 先看拓扑

```text
            ┌──────────────────────────────┐
            │  H3C AC / AP（业务网）        │
            │  VLAN 200：192.168.1.0/24    │
            └───┬───────────┬───────────┬──┘
       有线/5G │      5GHz │       有线 │
   ┌──────────▼───┐ ┌──────▼──────┐ ┌───▼──────────────┐
   │ 中枢地面站     │ │ SpiderPi Pro│ │ Minihomer 基站    │
   │ 192.168.1.100│ │ 192.168.1.101│ │ 192.168.1.123   │
   └──────────────┘ └─────────────┘ └──────────────────┘
```

## 2. 规划表

| 项目 | 值 |
|------|-----|
| 管理 VLAN | 100，`192.168.100.0/24` |
| 业务 VLAN | 200，`192.168.1.0/24` |
| 业务网关 | `192.168.1.1` |
| 5GHz SSID | `AGCS-5G` |
| 2.4GHz SSID | `AGCS-2.4G` |
| 安全 | WPA2-PSK / AES |
| 固定 IP | 地面站 `.100`、机器人 `.101`、Minihomer `.123`、图传 `.10` |

## 3. 准备工作

1. 记录 AP 序列号（设备标签或 `display device manuinfo`）；
2. 记录地面站和机器人的 MAC 地址；
3. AC/AP 上电，AP 用 PoE 交换机或 PoE 注入器供电；
4. 调试电脑临时设置 `192.168.100.10/24`，登录 AC/AP 管理口。

## 4. AC + Fit AP 配置

### 4.1 基础 VLAN 与网关

```text
system-view
sysname AGCS-AC
vlan 100
vlan 200
quit

interface Vlan-interface100
 ip address 192.168.100.1 255.255.255.0
 quit

interface Vlan-interface200
 ip address 192.168.1.1 255.255.255.0
 quit
```

### 4.2 DHCP 地址池与静态绑定

```text
dhcp enable
dhcp server ip-pool agcs-business
 network 192.168.1.0 mask 255.255.255.0
 gateway-list 192.168.1.1
 dns-list 223.5.5.5
 expired day 1
 static-bind ip-address 192.168.1.100 hardware-address <地面站MAC>
 static-bind ip-address 192.168.1.101 hardware-address <机器人MAC>
 quit
```

### 4.3 无线服务模板

```text
wlan service-template agcs-5g
 ssid AGCS-5G
 vlan 200
 security-ie wpa2
 cipher-suite ccmp
 akm mode psk
 preshared-key pass-phrase simple AGCS-2026
 service-template enable
 quit

wlan service-template agcs-24
 ssid AGCS-2.4G
 vlan 200
 security-ie wpa2
 cipher-suite ccmp
 akm mode psk
 preshared-key pass-phrase simple AGCS-2026
 service-template enable
 quit
```

### 4.4 AP 上线与射频绑定

```text
wlan ap ap1 model WA6320
 serial-id <AP序列号>
 quit

wlan ap ap1
 radio 1
  service-template agcs-5g
  radio enable
 radio 2
  service-template agcs-24
  radio enable
 quit

save force
```

> `radio 1` / `radio 2` 只是示例，实际用
> `display wlan ap all verbose` 查看射频编号和 AP 状态，再对应绑定。

## 5. Fat AP 配置（没有 AC 时）

没有 AC 时，把第 4 节的 VLAN、DHCP、无线服务模板直接配置在 AP 自身：

```text
system-view
sysname AGCS-AP1
vlan 100
vlan 200
quit

interface Vlan-interface100
 ip address 192.168.100.11 255.255.255.0
 quit

interface Vlan-interface200
 ip address 192.168.1.1 255.255.255.0
 quit
```

其余 DHCP 和 `service-template` 命令同第 4 节，并在 AP 上行口放通业务 VLAN：

```text
interface GigabitEthernet1/0/1
 port link-type trunk
 port trunk permit vlan 100 200
 quit
```

## 6. Web 配置路径（新手可选）

1. 浏览器登录 AC/AP 管理地址；
2. H3C：`无线网络 → 无线服务`，新建 `AGCS-5G`、`AGCS-2.4G`；
3. 网络：创建 VLAN、DHCP 地址池和静态绑定；
4. AP 管理：确认 AP 在线，绑定服务模板；
5. 保存并应用。

## 7. 配置后验证

```text
display wlan ap all
display wlan service-template
display dhcp server ip-in-use
display current-configuration
```

手机/笔记本连接 `AGCS-5G`，确认拿到 `192.168.1.x`；再从地面站执行：

```bat
ping 192.168.1.1
ping 192.168.1.101
ping 192.168.1.123
```

## 8. 常见问题

| 现象 | 排查 |
|------|------|
| AP 不上线 | PoE 供电、AC 与 AP 的 VLAN 放通、序列号是否正确 |
| 连上 SSID 拿不到 IP | DHCP 地址池、VLAN 创建与绑定、AP 是否透传业务 VLAN |
| 拿到 IP 但 ping 不通 | 网关、交换机端口 VLAN、无线隔离是否误开 |
| 命令报错 | 先 `display version` 确认平台，再查对应 H3C Comware 手册 |
| 配置丢失 | 使用 `save force` 保存；断电前确认保存 |
