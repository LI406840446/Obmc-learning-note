# OpenBMC Deep Wiki Learning Notes

This document contains notes and learnings about OpenBMC from Deep Wiki resources.
https://deepwiki.com/openbmc/docs/1-overview
## Table of Contents
- [Overview](#overview)
- [Key Concepts](#key-concepts)
- [Architecture](#architecture)
- [Development](#development)
- [Resources](#resources)

## Overview
<img width="1676" height="722" alt="image" src="https://github.com/user-attachments/assets/81e6876b-4800-4a5f-aefe-37a9431e300f" />  

```diff  
這張是基本架構圖，不同vendor會加入自己的service, daemon,dbus interface,OEM command等等。
整張圖可以分成 5 層
1. External Management Layer
2. Web and API Layer
3. Internal Communication Layer
4. Core Management Services
5. Hardware Interface Layer / Physical Hardware

!1.External Management Layer
也就是使用者或管理工具從外面連到 BMC。
這些都不是 BMC 內部 service，而是外部 client。
Web Browser
    → 開 iDRAC / OpenBMC Web GUI

Redfish Management Tools
    → 用 curl / python-redfish / automation tool

SSH Clients
    → ssh 進 BMC shell

IPMI Tools
    → ipmitool sensor list / chassis power status


!2. Web and API Layer
這層是 BMC 對外提供的入口。
-bmcweb HTTP Server
-webui-vue Application
-Phosphor D-Bus REST API
-Redfish REST API
-SSH Service (Dropbear)
-phosphor-host-ipmid

-bmcweb HTTP Server處理
HTTPS
Web GUI
Redfish API
REST API
外部 Web Browser 或 Redfish tool 進來，通常先打到 bmcweb

-webui-vue Application
使用者在瀏覽器看到的 OpenBMC Web 介面
它通常透過 bmcweb 呼叫 backend API。

-Redfish REST API
這是現代 BMC 管理標準。
GET /redfish/v1/Systems/system
GET /redfish/v1/Chassis/chassis/Sensors
POST /redfish/v1/Systems/system/Actions/ComputerSystem.Reset
它用 HTTP/JSON，方便 automation。

-Phosphor D-Bus REST API
這是把 D-Bus object 暴露成 REST API 的舊式/內部風格 API。

-SSH Service (Dropbear)
這是讓使用者 SSH 到 BMC shell。

-phosphor-host-ipmid
這是 IPMI daemon。
它處理 IPMI command，例如：
ipmitool sensor list
ipmitool chassis power status
ipmitool fru print


!3.Internal Communication Layer
-D-Bus Message Bus, phosphor-mapper是OpenBMC 架構最核心的部分
power service 想知道 sensor value
    → 去 D-Bus 找 sensor object

Redfish 想回傳 power state
    → 從 D-Bus 讀 host state object

IPMI 想讀 sensor
    → 從 D-Bus sensor object 轉成 IPMI response
所以 D-Bus 是 OpenBMC 內部的 IPC backbone。

其中phosphor-mapper 是 D-Bus object mapper。
它負責回答：
某個 D-Bus object path 是哪個 service 提供的？
某個 interface 在哪個 object 上？
例如：
我要找 /xyz/openbmc_project/sensors/temperature/cpu0
是誰提供？
phosphor-mapper 會告訴你 service name。

所以它像 OpenBMC 裡的 D-Bus directory service。


!4. Core Management Services
圖下方中間：
-phosphor-led-manager
-phosphor-inventory-manager
-phosphor-hwmon
-phosphor-logging
-phosphor-state-manager
-phosphor-user-manager
這些是核心管理服務。(最基本要有的service)

-phosphor-hwmon
負責硬體 sensor 讀值。
通常會讀：
temperature
voltage
fan tach
current
power
資料來源可能是：
/sys/class/hwmon/
I2C sensor driver
PMBus driver
然後 publish 成 D-Bus sensor object or probe reading。

-phosphor-inventory-manager負責 inventory。
例如：
PSU
DIMM
CPU
Fan
Board
Chassis
Drive
它會把硬體組件建立成 D-Bus inventory object。

-phosphor-logging
負責 log/event。

-phosphor-state-manager
負責狀態管理。
例如：
BMC state
Host state
Chassis power state
Boot progress

-phosphor-user-manager
負責 user/account。
phosphor-led-manager
負責 LED 控制。
例如：
identify LED
fault LED
status LED
通常根據 D-Bus event 或 fault 狀態控制 GPIO/LED。


!5. Hardware Interface Layer
圖下方偏右：
-systemd
-libmctp
-pldmtool
-phosphor-ipmi-host
這層是跟更底層或 host 溝通的介面。

-systemd
OpenBMC 是 embedded Linux，所以 service 啟動、管理、restart 通常由 systemd 控制。
例如：
systemctl status bmcweb
systemctl restart phosphor-hwmon
journalctl -u bmcweb

-libmctp
MCTP library。
用於和支援 MCTP 的 endpoint 溝通，例如：
CPU via PECI-over-MCTP-over-I3C
PCIe device
NVMe-MI
PLDM device

-phosphor-ipmi-host
這個比較偏 host-BMC IPMI 介面。
Host OS 可以透過：
KCS / BT / SSIF
/dev/ipmi0
和 BMC 溝通。

!6. Physical Hardware
圖左下角：
-Storage Devices
-Temperature/Voltage Sensors
-Cooling Fans
-Power Supplies
這些就是 BMC 實際管理的硬體。
它們通常透過：
I2C
SMBus
PMBus
GPIO
PWM/Tach
PECI
MCTP
PCIe
連到 BMC。
```




























## Key Concepts

Add key concepts and definitions here...

## Architecture

Add architecture notes here...

## Development

Add development-related notes here...

## Resources

Add resource links and references here...

