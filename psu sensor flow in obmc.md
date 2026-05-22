## OpenBMC PSU 運作流程
```diff
path - \ast2600\openbmc\as26_build\tmp\work\armv7ahf-vfpv4d16-openbmc-linux-gnueabi\dbus-sensors\0.1+git\sources\dbus-sensors-0.1+git\src\psu\PSUSensorMain.cpp

流程圖
[PSU 硬體]
   |
   | PMBus / I2C
   v
[Linux kernel driver]
   |
   | 產生 hwmon sysfs 節點
   v
/sys/class/hwmon/hwmonX/
   |
   | 例如:
   | in1_input
   | curr1_input
   | power1_input
   | fan1_input
   | temp1_input
   | *_label
   | *_alarm
   v
[dbus-sensors: psusensor]
   |
   | /usr/libexec/dbus-sensors/psusensor
   v
[D-Bus Sensor Objects]
   |
   | xyz.openbmc_project.Sensor.Value
   | xyz.openbmc_project.Sensor.Threshold.*
   | xyz.openbmc_project.State.Decorator.Availability
   v
[其他 OpenBMC service]
   |
   | bmcweb / IPMI / phosphor services
   v
[Redfish / IPMI / Web UI]



!1. systemd 啟動 PSU sensor service
Requires=xyz.openbmc_project.EntityManager.service
After=xyz.openbmc_project.EntityManager.service

Type=dbus
BusName=xyz.openbmc_project.PSUSensor
ExecStart=/usr/libexec/dbus-sensors/psusensor
意思是：
PSU sensor 依賴 EntityManager
EntityManager 先提供硬體設定
psusensor 啟動後註冊 D-Bus service
D-Bus name 是：xyz.openbmc_project.PSUSensor


!2. EntityManager 提供 PSU 設定
OpenBMC 通常不是 psusensor 自己硬寫 PSU 在哪裡，而是由 EntityManager 提供設定。
{
  "Name": "PSU1",
  "Type": "pmbus",
  "Bus": 3,
  "Address": "0x58"
}
可能放在：
entity-manager config
platform YAML
machine-specific config

openbmc的平台 layer 可能在：
meta-ieisystem/
meta-aspeed-sdk/

這些設定會告訴系統：
PSU 在哪條 I2C bus
I2C address 是多少
PSU chip type 是什麼
Sensor 名稱是什麼
threshold 怎麼設定

!3. Linux kernel 透過 PMBus / hwmon 讀 PSU
PSU 常見是 PMBus 裝置，接在 I2C 上。
Kernel driver bind 成功後，會產生 hwmon 節點，例如：
/sys/class/hwmon/hwmonX/
裡面可能有：
in1_input
in1_label
curr1_input
curr1_label
power1_input
power1_label
fan1_input
fan1_label
temp1_input
temp1_label

!4. PSUSensorMain.cpp 掃描 PSU 支援的 chip type
這表示 psusensor 會依據 EntityManager 設定或 I2C 裝置資訊，找到對應的 Linux driver/device type。
{"ADM1272", I2CDeviceType{"adm1272", true}},
{"ADM1275", I2CDeviceType{"adm1275", true}},
{"ADM1278", I2CDeviceType{"adm1278", true}},
{"cffps", I2CDeviceType{"cffps", true}},
{"INA219", I2CDeviceType{"ina219", true}},
{"INA226", I2CDeviceType{"ina226", true}},
{"INA233", I2CDeviceType{"ina233", true}},
{"LM25066", I2CDeviceType{"lm25066", true}},
{"pmbus", I2CDeviceType{"pmbus", true}},

!5. psusensor 把 hwmon 檔案轉成 OpenBMC Sensor
static constexpr const std::array<SensorUnit, 6> sensorTable{{
    {"curr", sensor_paths::unitAmperes},
    {"fan", sensor_paths::unitRPMs},
    {"in", sensor_paths::unitVolts},
    {"power", sensor_paths::unitWatts},
    {"temp", sensor_paths::unitDegreesC},
    {"voltage", sensor_paths::unitVolts},
}};
這代表它會把 hwmon sensor 類型轉成 OpenBMC sensor 單位：

hwmon 類型	    OpenBMC 單位
curr	          Ampere
fan	            RPM
in	            Volt
power	          Watt
temp	          Celsius
voltage	        Volt

所以 PSU 的資料會變成類似：
/xyz/openbmc_project/sensors/voltage/PSU1_Output_Voltage
/xyz/openbmc_project/sensors/current/PSU1_Output_Current
/xyz/openbmc_project/sensors/power/PSU1_Output_Power
/xyz/openbmc_project/sensors/fan_tach/PSU1_Fan_Speed
/xyz/openbmc_project/sensors/temperature/PSU1_Temperature
實際名稱要看你的平台設定與 label。

!6. Label 對應邏輯
PSUSensorMain.cpp 裡有 labelMatch 表，例如：
{"iin", "Input Current", 20, 0, 3, 0},
{"iout", "Output Current", 255, 0, 3, 0},
{"pin", "Input Power", 3000, 0, 6, 0},
{"pout", "Output Power", 3000, 0, 6, 0},
{"temp", "Temperature", 127, -128, 3, 0},
{"vin", "Input Voltage", 300, 0, 3, 0},
{"vout", "Output Voltage", 255, 0, 3, 0},
它會根據 hwmon label 判斷這個 sensor 是什麼。
vout -> Output Voltage
iout -> Output Current
pin  -> Input Power
pout -> Output Power
temp -> Temperature

!7. PSUSensor 週期性讀取資料
PSUSensor.hpp 裡可以看到：class PSUSensor : public Sensor
它繼承 OpenBMC dbus-sensors 的 Sensor 基底類別。
重要 function：
void setupRead();
void activate(...);
void deactivate();
bool isActive();
void handleResponse(...);
void checkThresholds() override;
還有預設 poll rate：
static constexpr double defaultSensorPoll = 1.0;
static constexpr unsigned int defaultSensorPollMs =
    static_cast<unsigned int>(defaultSensorPoll * 1000);

!8. PSU event / alarm
除了數值型 sensor，PSU 還有 event / alarm。
const static EventPathList eventMatch{
    {"PredictiveFailure", {"power1_alarm"}},
    {"Failure", {"in2_alarm"}},
    {"ACLost", {"in1_beep"}},
    ...
};
這表示它會看 hwmon 裡的 alarm/beep 類檔案，例如
power1_alarm
in2_alarm
in1_beep
然後轉成 PSU event，例如：
PredictiveFailure
Failure
ACLost
這部分邏輯在：
src/psu/PSUEvent.cpp
src/psu/PSUEvent.hpp

!9. D-Bus 上最後會長什麼樣子
PSU sensor 最後會變成 D-Bus object。
/xyz/openbmc_project/sensors/power/PSU1_Input_Power
/xyz/openbmc_project/sensors/power/PSU1_Output_Power
/xyz/openbmc_project/sensors/voltage/PSU1_Input_Voltage
/xyz/openbmc_project/sensors/voltage/PSU1_Output_Voltage
/xyz/openbmc_project/sensors/current/PSU1_Output_Current
/xyz/openbmc_project/sensors/temperature/PSU1_Temperature
/xyz/openbmc_project/sensors/fan_tach/PSU1_Fan_Speed

常見 interface：
xyz.openbmc_project.Sensor.Value
xyz.openbmc_project.Sensor.Threshold.Warning
xyz.openbmc_project.Sensor.Threshold.Critical
xyz.openbmc_project.State.Decorator.Availability
xyz.openbmc_project.Association.Definitions

!10. Redfish 怎麼看到 PSU
他的endpoint會去對照相對應的dbus path
```

##PSU 啟動到顯示完整流程
```diff
1. BMC boot
   |
   v
2. kernel 初始化 I2C / PMBus driver
   |
   v
3. PSU device bind 到 hwmon
   |
   v
4. /sys/class/hwmon/hwmonX 出現 PSU sensor 檔案
   |
   v
5. EntityManager 載入平台 PSU config
   |
   v
6. systemd 啟動 xyz.openbmc_project.psusensor.service
   |
   v
7. /usr/libexec/dbus-sensors/psusensor 啟動
   |
   v
8. psusensor 從 EntityManager 拿 PSU 設定
   |
   v
9. psusensor 掃描對應 hwmon path
   |
   v
10. 建立 D-Bus sensor object
   |
   v
11. 每秒讀 sysfs value
   |
   v
12. 更新 D-Bus Sensor.Value
   |
   v
13. 檢查 threshold / alarm
   |
   v
14. bmcweb / IPMI / WebUI 從 D-Bus 讀資料







createSensorsCallback()
 |
 |-- 從 EntityManager config 建立 I2C device 管理物件
 |
 |-- 掃描 /sys/bus/iio/devices 與 /sys/class/hwmon
 |
 |-- 讀每個 hwmon/iio 裝置的 name
 |
 |-- 確認 name 是否在 sensorTypes 白名單
 |
 |-- 從 sysfs 路徑解析 I2C bus/address
 |
 |-- 用 bus/address 去 EntityManager config 裡找對應 PSU
 |
 |-- 讀 PSU 名稱、threshold、poll rate、Labels
 |
 |-- 掃描 input/max sensor 檔案
 |
 |-- 讀 label，例如 vin/vout/iout/pin/pout/temp/fan
 |
 |-- 依 labelMatch 決定 sensor 名稱、單位、scale、min/max
 |
 |-- 建立 PSUSensor
 |
 |-- setupRead() 開始週期性讀 sysfs
 |
 |-- 建立 PSUCombineEvent 處理 OperationalStatus
```


















































