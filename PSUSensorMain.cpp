這份cpp的重點
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



// NOTE: Reading-only annotated copy of PSUSensorMain.cpp. Do not build this file.
// NOTE: Original OpenBMC source is unchanged. Chinese comments explain each line/block.

/* // 授權或檔頭註解開始。
// Copyright (c) 2019 Intel Corporation // 授權或檔頭註解內容。
// // 授權或檔頭註解內容。
// Licensed under the Apache License, Version 2.0 (the "License"); // 授權或檔頭註解內容。
// you may not use this file except in compliance with the License. // 授權或檔頭註解內容。
// You may obtain a copy of the License at // 授權或檔頭註解內容。
// // 授權或檔頭註解內容。
//      http://www.apache.org/licenses/LICENSE-2.0 // 授權或檔頭註解內容。
// // 授權或檔頭註解內容。
// Unless required by applicable law or agreed to in writing, software // 授權或檔頭註解內容。
// distributed under the License is distributed on an "AS IS" BASIS, // 授權或檔頭註解內容。
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. // 授權或檔頭註解內容。
// See the License for the specific language governing permissions and // 授權或檔頭註解內容。
// limitations under the License. // 授權或檔頭註解內容。
*/ // 授權或檔頭註解結束。

#include "DeviceMgmt.hpp" // 引入此檔需要使用的標頭/函式庫。
#include "PSUEvent.hpp" // 引入此檔需要使用的標頭/函式庫。
#include "PSUSensor.hpp" // 引入此檔需要使用的標頭/函式庫。
#include "PwmSensor.hpp" // 引入此檔需要使用的標頭/函式庫。
#include "SensorPaths.hpp" // 引入此檔需要使用的標頭/函式庫。
#include "Thresholds.hpp" // 引入此檔需要使用的標頭/函式庫。
#include "Utils.hpp" // 引入此檔需要使用的標頭/函式庫。
#include "VariantVisitors.hpp" // 引入此檔需要使用的標頭/函式庫。

#include <boost/algorithm/string/case_conv.hpp> // 引入此檔需要使用的標頭/函式庫。
#include <boost/algorithm/string/replace.hpp> // 引入此檔需要使用的標頭/函式庫。
#include <boost/asio/error.hpp> // 引入此檔需要使用的標頭/函式庫。
#include <boost/asio/io_context.hpp> // 引入此檔需要使用的標頭/函式庫。
#include <boost/asio/post.hpp> // 引入此檔需要使用的標頭/函式庫。
#include <boost/asio/steady_timer.hpp> // 引入此檔需要使用的標頭/函式庫。
#include <boost/container/flat_map.hpp> // 引入此檔需要使用的標頭/函式庫。
#include <boost/container/flat_set.hpp> // 引入此檔需要使用的標頭/函式庫。
#include <phosphor-logging/lg2.hpp> // 引入此檔需要使用的標頭/函式庫。
#include <sdbusplus/asio/connection.hpp> // 引入此檔需要使用的標頭/函式庫。
#include <sdbusplus/asio/object_server.hpp> // 引入此檔需要使用的標頭/函式庫。
#include <sdbusplus/bus.hpp> // 引入此檔需要使用的標頭/函式庫。
#include <sdbusplus/bus/match.hpp> // 引入此檔需要使用的標頭/函式庫。
#include <sdbusplus/exception.hpp> // 引入此檔需要使用的標頭/函式庫。
#include <sdbusplus/message.hpp> // 引入此檔需要使用的標頭/函式庫。
#include <sdbusplus/message/native_types.hpp> // 引入此檔需要使用的標頭/函式庫。

#include <algorithm> // 引入此檔需要使用的標頭/函式庫。
#include <array> // 引入此檔需要使用的標頭/函式庫。
#include <cctype> // 引入此檔需要使用的標頭/函式庫。
#include <chrono> // 引入此檔需要使用的標頭/函式庫。
#include <cmath> // 引入此檔需要使用的標頭/函式庫。
#include <cstddef> // 引入此檔需要使用的標頭/函式庫。
#include <cstdint> // 引入此檔需要使用的標頭/函式庫。
#include <exception> // 引入此檔需要使用的標頭/函式庫。
#include <filesystem> // 引入此檔需要使用的標頭/函式庫。
#include <format> // 引入此檔需要使用的標頭/函式庫。
#include <fstream> // 引入此檔需要使用的標頭/函式庫。
#include <functional> // 引入此檔需要使用的標頭/函式庫。
#include <iterator> // 引入此檔需要使用的標頭/函式庫。
#include <memory> // 引入此檔需要使用的標頭/函式庫。
#include <ranges> // 引入此檔需要使用的標頭/函式庫。
#include <regex> // 引入此檔需要使用的標頭/函式庫。
#include <stdexcept> // 引入此檔需要使用的標頭/函式庫。
#include <string> // 引入此檔需要使用的標頭/函式庫。
#include <string_view> // 引入此檔需要使用的標頭/函式庫。
#include <utility> // 引入此檔需要使用的標頭/函式庫。
#include <variant> // 引入此檔需要使用的標頭/函式庫。
#include <vector> // 引入此檔需要使用的標頭/函式庫。

static std::regex devRegex(R"((\/i[23]c\-\d+\/\d+-[a-fA-F0-9]{4,4})(\/|$))"); // 定義 regex，用來從 sysfs device path 擷取 I2C bus/address。

static constexpr auto sensorTypes = std::to_array< // PSU sensor 支援的 EntityManager 型別與 kernel driver 名稱白名單。
    std::pair<std::string_view, I2CDeviceType>>({ // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"ADC128D818", I2CDeviceType{"adc128d818", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"ADM1266", I2CDeviceType{"adm1266", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"ADM1272", I2CDeviceType{"adm1272", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"ADM1275", I2CDeviceType{"adm1275", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"ADM1278", I2CDeviceType{"adm1278", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"ADM1281", I2CDeviceType{"adm1281", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"ADM1293", I2CDeviceType{"adm1293", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"ADS1015", I2CDeviceType{"ads1015", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"ADS7830", I2CDeviceType{"ads7830", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"AHE50DC_FAN", I2CDeviceType{"ahe50dc_fan", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"BMR490", I2CDeviceType{"bmr490", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"cffps", I2CDeviceType{"cffps", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"cffps1", I2CDeviceType{"cffps", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"cffps2", I2CDeviceType{"cffps", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"cffps3", I2CDeviceType{"cffps", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"CRPS185", I2CDeviceType{"crps185", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"DPS800", I2CDeviceType{"dps800", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"INA219", I2CDeviceType{"ina219", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"INA226", I2CDeviceType{"ina226", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"INA230", I2CDeviceType{"ina230", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"INA233", I2CDeviceType{"ina233", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"INA238", I2CDeviceType{"ina238", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"IPSPS1", I2CDeviceType{"ipsps1", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"IR35221", I2CDeviceType{"ir35221", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"IR38060", I2CDeviceType{"ir38060", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"IR38164", I2CDeviceType{"ir38164", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"IR38263", I2CDeviceType{"ir38263", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"ISL28022", I2CDeviceType{"isl28022", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"ISL68137", I2CDeviceType{"isl68137", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"ISL68220", I2CDeviceType{"isl68220", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"ISL68223", I2CDeviceType{"isl68223", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"ISL69225", I2CDeviceType{"isl69225", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"ISL69243", I2CDeviceType{"isl69243", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"ISL69260", I2CDeviceType{"isl69260", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"LM25066", I2CDeviceType{"lm25066", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"LM5066I", I2CDeviceType{"lm5066i", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"LTC2945", I2CDeviceType{"ltc2945", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"LTC4286", I2CDeviceType{"ltc4286", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"LTC4287", I2CDeviceType{"ltc4287", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MAX5970", I2CDeviceType{"max5970", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MAX11607", I2CDeviceType{"max11607", false}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MAX11615", I2CDeviceType{"max11615", false}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MAX11617", I2CDeviceType{"max11617", false}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MAX16601", I2CDeviceType{"max16601", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MAX20710", I2CDeviceType{"max20710", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MAX20730", I2CDeviceType{"max20730", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MAX20734", I2CDeviceType{"max20734", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MAX20796", I2CDeviceType{"max20796", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MAX34451", I2CDeviceType{"max34451", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MP2856", I2CDeviceType{"mp2856", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MP2857", I2CDeviceType{"mp2857", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MP2869", I2CDeviceType{"mp2869", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MP2971", I2CDeviceType{"mp2971", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MP2973", I2CDeviceType{"mp2973", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MP2975", I2CDeviceType{"mp2975", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MP2993", I2CDeviceType{"mp2993", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MP5023", I2CDeviceType{"mp5023", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MP5926", I2CDeviceType{"mp5926", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MP5990", I2CDeviceType{"mp5990", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MP5998", I2CDeviceType{"mp5998", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MP9945", I2CDeviceType{"mp9945", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MP29612", I2CDeviceType{"mp29612", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"MPQ8785", I2CDeviceType{"mpq8785", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"NCP4200", I2CDeviceType{"ncp4200", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"PLI1209BC", I2CDeviceType{"pli1209bc", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"pmbus", I2CDeviceType{"pmbus", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"PXE1610", I2CDeviceType{"pxe1610", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"SQ52206", I2CDeviceType{"sq52206", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"RAA228000", I2CDeviceType{"raa228000", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"RAA228004", I2CDeviceType{"raa228004", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"RAA228006", I2CDeviceType{"raa228006", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"RAA228228", I2CDeviceType{"raa228228", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"RAA228620", I2CDeviceType{"raa228620", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"RAA229001", I2CDeviceType{"raa229001", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"RAA229004", I2CDeviceType{"raa229004", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"RAA229126", I2CDeviceType{"raa229126", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"RTQ6056", I2CDeviceType{"rtq6056", false}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"SBRMI", I2CDeviceType{"sbrmi", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"smpro_hwmon", I2CDeviceType{"smpro", false}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"SY24655", I2CDeviceType{"sy24655", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"TDA38640", I2CDeviceType{"tda38640", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"TPS25990", I2CDeviceType{"tps25990", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"TPS53679", I2CDeviceType{"tps53679", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"TPS546D24", I2CDeviceType{"tps546d24", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"XDP710", I2CDeviceType{"xdp710", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"XDPE11280", I2CDeviceType{"xdpe11280", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"XDPE12284", I2CDeviceType{"xdpe12284", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
    {"XDPE152C4", I2CDeviceType{"xdpe152c4", true}}, // 把設定檔中的 PSU 型別對應到 Linux I2C/hwmon driver 名稱。
}); // 程式區塊或初始化列表的結構符號。

enum class DevTypes // 定義支援的 Linux sensor 來源類型。
{ // 程式區塊或初始化列表的結構符號。
    Unknown = 0, // sensor 來源類型列舉值。
    HWMON, // sensor 來源類型列舉值。
    IIO // sensor 來源類型列舉值。
}; // 程式區塊或初始化列表的結構符號。

struct DevParams // 描述不同 sensor 來源要如何用 regex 解析檔名。
{ // 程式區塊或初始化列表的結構符號。
    unsigned int matchIndex = 0; // 用於解析 sysfs sensor 檔名的參數。
    std::string_view matchRegEx; // 用於解析 sysfs sensor 檔名的參數。
    std::string_view nameRegEx; // 用於解析 sysfs sensor 檔名的參數。
}; // 程式區塊或初始化列表的結構符號。

static boost::container::flat_map<std::string, std::shared_ptr<PSUSensor>> // 程式結構/變數設定，配合上下文理解。
    sensors; // 全域表：保存已建立的 PSU sensor 物件。
static boost::container::flat_map<std::string, std::unique_ptr<PSUCombineEvent>> // 建立 PSU OperationalStatus 事件聚合物件。
    combineEvents; // 全域表：保存 PSU alarm/event 聚合物件。
static boost::container::flat_map<std::string, std::unique_ptr<PwmSensor>> // 程式結構/變數設定，配合上下文理解。
    pwmSensors; // 全域表：保存 PSU fan PWM 控制物件。

struct SensorUnit // 定義 sensor 類型名稱與 OpenBMC 單位的對應結構。
{ // 程式區塊或初始化列表的結構符號。
    std::string_view name; // 程式結構/變數設定，配合上下文理解。
    std::string_view units; // 程式結構/變數設定，配合上下文理解。
    auto operator<=>(const SensorUnit&) const = default; // 程式結構/變數設定，配合上下文理解。
}; // 程式區塊或初始化列表的結構符號。

static constexpr const std::array<SensorUnit, 6> sensorTable{{ // 把 hwmon/IIO 類型對應到 D-Bus sensor unit。
    {"curr", sensor_paths::unitAmperes}, // 電流 sensor 單位為 Ampere。
    {"fan", sensor_paths::unitRPMs}, // 風扇轉速 sensor 單位為 RPM。
    {"in", sensor_paths::unitVolts}, // 電壓 sensor 單位為 Volt。
    {"power", sensor_paths::unitWatts}, // 功率 sensor 單位為 Watt。
    {"temp", sensor_paths::unitDegreesC}, // 溫度 sensor 單位為 Celsius。
    {"voltage", sensor_paths::unitVolts}, // 電壓 sensor 單位為 Volt。
}}; // 程式區塊或初始化列表的結構符號。

static constexpr std::array<PSUProperty, 20> labelMatch{{ // 定義 hwmon label 與可讀名稱、範圍、scale、offset 的預設對應。
    {"curr", "Output Current", 255, 0, 3, 0}, // 程式結構/變數設定，配合上下文理解。
    {"fan", "Fan Speed ", 30000, 0, 0, 0}, // 程式結構/變數設定，配合上下文理解。
    {"iin", "Input Current", 20, 0, 3, 0}, // 程式結構/變數設定，配合上下文理解。
    {"in_voltage", "Output Voltage", 255, 0, 3, 0}, // 程式結構/變數設定，配合上下文理解。
    {"in", "Output Voltage", 255, 0, 3, 0}, // 程式結構/變數設定，配合上下文理解。
    {"iout", "Output Current", 255, 0, 3, 0}, // 程式結構/變數設定，配合上下文理解。
    {"maxiout", "Max Output Current", 255, 0, 3, 0}, // 程式結構/變數設定，配合上下文理解。
    {"maxpin", "Max Input Power", 3000, 0, 6, 0}, // 程式結構/變數設定，配合上下文理解。
    {"maxtemp", "Max Temperature", 127, -128, 3, 0}, // 程式結構/變數設定，配合上下文理解。
    {"maxvin", "Max Input Voltage", 300, 0, 3, 0}, // 程式結構/變數設定，配合上下文理解。
    {"pin", "Input Power", 3000, 0, 6, 0}, // 程式結構/變數設定，配合上下文理解。
    {"pout", "Output Power", 3000, 0, 6, 0}, // 程式結構/變數設定，配合上下文理解。
    {"power", "Output Power", 3000, 0, 6, 0}, // 程式結構/變數設定，配合上下文理解。
    {"temp", "Temperature", 127, -128, 3, 0}, // 程式結構/變數設定，配合上下文理解。
    {"vin", "Input Voltage", 300, 0, 3, 0}, // 程式結構/變數設定，配合上下文理解。
    {"vmon", "Auxiliary Input Voltage", 255, 0, 3, 0}, // 程式結構/變數設定，配合上下文理解。
    {"voltage", "Output Voltage", 255, 0, 3, 0}, // 程式結構/變數設定，配合上下文理解。
    {"vout", "Output Voltage", 255, 0, 3, 0}, // 程式結構/變數設定，配合上下文理解。
}}; // 程式區塊或初始化列表的結構符號。

const static EventPathList eventMatch{{"PredictiveFailure", {"power1_alarm"}}, // 定義 PSU 狀態事件與 sysfs alarm/beep 檔案的對應。
                                      {"Failure", {"in2_alarm"}}, // 程式結構/變數設定，配合上下文理解。
                                      {"ACLost", {"in1_beep"}}, // 程式結構/變數設定，配合上下文理解。
                                      {"ConfigureError", {"in1_fault"}}}; // 程式結構/變數設定，配合上下文理解。
const static EventPathList limitEventMatch{ // 定義 threshold alarm 類事件與 sysfs alarm 檔名後綴的對應。
    {"PredictiveFailure", {"max_alarm", "min_alarm"}}, // 程式結構/變數設定，配合上下文理解。
    {"Failure", {"crit_alarm", "lcrit_alarm"}}}; // 程式結構/變數設定，配合上下文理解。

static boost::container::flat_map<size_t, bool> cpuPresence; // 記錄 CPU 是否存在，用來決定某些 sensor 是否要建立。
constexpr static auto devParamMap = // 依 HWMON/IIO 分別定義 sensor 檔案搜尋與名稱解析規則。
    std::to_array<std::pair<DevTypes, DevParams>>( // 程式結構/變數設定，配合上下文理解。
        {{DevTypes::HWMON, {1, R"(\w\d+_input$)", "([A-Za-z]+)[0-9]*_"}}, // sensor 來源類型列舉值。
         {DevTypes::IIO, // sensor 來源類型列舉值。
          {2, R"(\w+_(raw|input)$)", "^(in|out)_([A-Za-z]+)[0-9]*_"}}}); // 程式結構/變數設定，配合上下文理解。

// Function CheckEvent will check each attribute from eventMatch table in the // 原始碼內既有說明註解。
// sysfs. If the attributes exists in sysfs, then store the complete path // 原始碼內既有說明註解。
// of the attribute into eventPathList. // 原始碼內既有說明註解。
void checkEvent(const std::string& directory, const EventPathList& eventMatch, // 定義 PSU 狀態事件與 sysfs alarm/beep 檔案的對應。
                EventPathList& eventPathList) // 程式結構/變數設定，配合上下文理解。
{ // 程式區塊或初始化列表的結構符號。
    for (const auto& match : eventMatch) // 定義 PSU 狀態事件與 sysfs alarm/beep 檔案的對應。
    { // 程式區塊或初始化列表的結構符號。
        const std::vector<std::string>& eventAttrs = match.second; // 程式結構/變數設定，配合上下文理解。
        const std::string& eventName = match.first; // 程式結構/變數設定，配合上下文理解。
        for (const auto& eventAttr : eventAttrs) // 迴圈：逐一處理集合中的元素。
        { // 程式區塊或初始化列表的結構符號。
            std::string eventPath = directory; // 程式結構/變數設定，配合上下文理解。
            eventPath += "/"; // 程式結構/變數設定，配合上下文理解。
            eventPath += eventAttr; // 程式結構/變數設定，配合上下文理解。

            std::ifstream eventFile(eventPath); // 程式結構/變數設定，配合上下文理解。
            if (!eventFile.good()) // 條件判斷，決定是否進入此分支。
            { // 程式區塊或初始化列表的結構符號。
                continue; // 跳過目前迴圈項目，處理下一個。
            } // 程式區塊或初始化列表的結構符號。

            eventPathList[eventName].push_back(eventPath); // 程式結構/變數設定，配合上下文理解。
        } // 程式區塊或初始化列表的結構符號。
    } // 程式區塊或初始化列表的結構符號。
} // 程式區塊或初始化列表的結構符號。

// Check Group Events which contains more than one targets in each combine // 原始碼內既有說明註解。
// events. // 原始碼內既有說明註解。
void checkGroupEvent(const std::string& directory, // 函式：檢查需合併多個 sysfs 屬性的群組事件，例如 fan fault。
                     GroupEventPathList& groupEventPathList) // 程式結構/變數設定，配合上下文理解。
{ // 程式區塊或初始化列表的結構符號。
    EventPathList pathList; // 程式結構/變數設定，配合上下文理解。
    std::vector<std::filesystem::path> eventPaths; // 程式結構/變數設定，配合上下文理解。
    if (!findFiles(std::filesystem::path(directory), R"(fan\d+_(alarm|fault))", // 在指定 sysfs 目錄搜尋符合 regex 的檔案。
                   eventPaths)) // 程式結構/變數設定，配合上下文理解。
    { // 程式區塊或初始化列表的結構符號。
        return; // 結束目前函式或提前離開。
    } // 程式區塊或初始化列表的結構符號。

    for (const auto& eventPath : eventPaths) // 迴圈：逐一處理集合中的元素。
    { // 程式區塊或初始化列表的結構符號。
        std::string attrName = eventPath.filename(); // 程式結構/變數設定，配合上下文理解。
        pathList[attrName.substr(0, attrName.find('_'))].push_back(eventPath); // 程式結構/變數設定，配合上下文理解。
    } // 程式區塊或初始化列表的結構符號。
    groupEventPathList["FanFault"] = pathList; // 程式結構/變數設定，配合上下文理解。
} // 程式區塊或初始化列表的結構符號。

// Function checkEventLimits will check all the psu related xxx_input attributes // 原始碼內既有說明註解。
// in sysfs to see if xxx_crit_alarm xxx_lcrit_alarm xxx_max_alarm // 原始碼內既有說明註解。
// xxx_min_alarm exist, then store the existing paths of the alarm attributes // 原始碼內既有說明註解。
// to eventPathList. // 原始碼內既有說明註解。
void checkEventLimits(const std::string& sensorPathStr, // 函式：檢查 PSU 一般事件檔案是否存在。
                      const EventPathList& limitEventMatch, // 定義 threshold alarm 類事件與 sysfs alarm 檔名後綴的對應。
                      EventPathList& eventPathList) // 程式結構/變數設定，配合上下文理解。
{ // 程式區塊或初始化列表的結構符號。
    auto attributePartPos = sensorPathStr.find_last_of('_'); // 程式結構/變數設定，配合上下文理解。
    if (attributePartPos == std::string::npos) // 條件判斷，決定是否進入此分支。
    { // 程式區塊或初始化列表的結構符號。
        // There is no '_' in the string, skip it // 原始碼內既有說明註解。
        return; // 結束目前函式或提前離開。
    } // 程式區塊或初始化列表的結構符號。
    auto attributePart = // 程式結構/變數設定，配合上下文理解。
        std::string_view(sensorPathStr).substr(attributePartPos + 1); // 程式結構/變數設定，配合上下文理解。
    if (attributePart != "input") // 條件判斷，決定是否進入此分支。
    { // 程式區塊或初始化列表的結構符號。
        // If the sensor is not xxx_input, skip it // 原始碼內既有說明註解。
        return; // 結束目前函式或提前離開。
    } // 程式區塊或初始化列表的結構符號。

    auto prefixPart = sensorPathStr.substr(0, attributePartPos + 1); // 程式結構/變數設定，配合上下文理解。
    for (const auto& limitMatch : limitEventMatch) // 定義 threshold alarm 類事件與 sysfs alarm 檔名後綴的對應。
    { // 程式區塊或初始化列表的結構符號。
        const std::vector<std::string>& limitEventAttrs = limitMatch.second; // 程式結構/變數設定，配合上下文理解。
        const std::string& eventName = limitMatch.first; // 程式結構/變數設定，配合上下文理解。
        for (const auto& limitEventAttr : limitEventAttrs) // 迴圈：逐一處理集合中的元素。
        { // 程式區塊或初始化列表的結構符號。
            auto limitEventPath = prefixPart + limitEventAttr; // 程式結構/變數設定，配合上下文理解。
            std::ifstream eventFile(limitEventPath); // 程式結構/變數設定，配合上下文理解。
            if (!eventFile.good()) // 條件判斷，決定是否進入此分支。
            { // 程式區塊或初始化列表的結構符號。
                continue; // 跳過目前迴圈項目，處理下一個。
            } // 程式區塊或初始化列表的結構符號。
            eventPathList[eventName].push_back(limitEventPath); // 程式結構/變數設定，配合上下文理解。
        } // 程式區塊或初始化列表的結構符號。
    } // 程式區塊或初始化列表的結構符號。
} // 程式區塊或初始化列表的結構符號。

static void checkPWMSensor( // 函式：若 fan input 有對應 target，建立 PSU PWM 控制 sensor。
    const std::filesystem::path& sensorPath, std::string& labelHead, // hwmon label 的主要名稱，例如 vin/vout/iout/fan/temp。
    const std::string& interfacePath, // EntityManager config object path，用來關聯 sensor 與設定。
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection, // 建立 system bus D-Bus 連線。
    sdbusplus::asio::object_server& objectServer, const std::string& psuName) // 建立 D-Bus object server，用來 expose sensor objects。
{ // 程式區塊或初始化列表的結構符號。
    if (!labelHead.starts_with("fan")) // hwmon label 的主要名稱，例如 vin/vout/iout/fan/temp。
    { // 程式區塊或初始化列表的結構符號。
        return; // 結束目前函式或提前離開。
    } // 程式區塊或初始化列表的結構符號。
    std::string labelHeadIndex = labelHead.substr(3); // hwmon label 的主要名稱，例如 vin/vout/iout/fan/temp。

    const std::string& sensorPathStr = sensorPath.string(); // 程式結構/變數設定，配合上下文理解。
    const std::string& pwmPathStr = // 程式結構/變數設定，配合上下文理解。
        boost::replace_all_copy(sensorPathStr, "input", "target"); // 程式結構/變數設定，配合上下文理解。
    std::ifstream pwmFile(pwmPathStr); // 程式結構/變數設定，配合上下文理解。
    if (!pwmFile.good()) // 條件判斷，決定是否進入此分支。
    { // 程式區塊或初始化列表的結構符號。
        return; // 結束目前函式或提前離開。
    } // 程式區塊或初始化列表的結構符號。

    auto findPWMSensor = pwmSensors.find(psuName + labelHead); // 全域表：保存 PSU fan PWM 控制物件。
    if (findPWMSensor != pwmSensors.end()) // 全域表：保存 PSU fan PWM 控制物件。
    { // 程式區塊或初始化列表的結構符號。
        return; // 結束目前函式或提前離開。
    } // 程式區塊或初始化列表的結構符號。

    std::string name = "Pwm_"; // 程式結構/變數設定，配合上下文理解。
    name += psuName; // 程式結構/變數設定，配合上下文理解。
    name += "_Fan_"; // 程式結構/變數設定，配合上下文理解。
    name += labelHeadIndex; // hwmon label 的主要名稱，例如 vin/vout/iout/fan/temp。

    std::string objPath = interfacePath; // EntityManager config object path，用來關聯 sensor 與設定。
    objPath += "_Fan_"; // 程式結構/變數設定，配合上下文理解。
    objPath += labelHeadIndex; // hwmon label 的主要名稱，例如 vin/vout/iout/fan/temp。

    pwmSensors[psuName + labelHead] = std::make_unique<PwmSensor>( // 全域表：保存 PSU fan PWM 控制物件。
        name, pwmPathStr, dbusConnection, objectServer, objPath, "PSU"); // 程式結構/變數設定，配合上下文理解。
} // 程式區塊或初始化列表的結構符號。

static void createSensorsCallback( // 核心函式：根據 EntityManager config 與 sysfs 建立 PSU sensors。
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer, // Boost.Asio event loop，用於非同步 D-Bus/timer 操作。
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection, // 建立 system bus D-Bus 連線。
    const ManagedObjectType& sensorConfigs, // EntityManager 回傳的 sensor 設定集合。
    const std::shared_ptr<boost::container::flat_set<std::string>>& // 程式結構/變數設定，配合上下文理解。
        sensorsChanged, // 處理 EntityManager 設定變更時，只重建受影響的 sensor。
    bool activateOnly) // 程式結構/變數設定，配合上下文理解。
{ // 程式區塊或初始化列表的結構符號。
    int numCreated = 0; // 程式結構/變數設定，配合上下文理解。
    bool firstScan = sensorsChanged == nullptr; // 處理 EntityManager 設定變更時，只重建受影響的 sensor。

    auto devices = instantiateDevices(sensorConfigs, sensors, sensorTypes); // PSU sensor 支援的 EntityManager 型別與 kernel driver 名稱白名單。

    std::vector<std::filesystem::path> pmbusPaths; // 程式結構/變數設定，配合上下文理解。
    findFiles(std::filesystem::path("/sys/bus/iio/devices"), "name", // sensor 來源類型列舉值。
              pmbusPaths); // 程式結構/變數設定，配合上下文理解。
    findFiles(std::filesystem::path("/sys/class/hwmon"), "name", pmbusPaths); // sensor 來源類型列舉值。
    if (pmbusPaths.empty()) // 條件判斷，決定是否進入此分支。
    { // 程式區塊或初始化列表的結構符號。
        lg2::error("No PSU sensors in system"); // 記錄錯誤訊息。
        return; // 結束目前函式或提前離開。
    } // 程式區塊或初始化列表的結構符號。

    boost::container::flat_set<std::string> directories; // 程式結構/變數設定，配合上下文理解。
    for (const auto& pmbusPath : pmbusPaths) // 迴圈：逐一處理集合中的元素。
    { // 程式區塊或初始化列表的結構符號。
        EventPathList eventPathList; // 程式結構/變數設定，配合上下文理解。
        GroupEventPathList groupEventPathList; // 程式結構/變數設定，配合上下文理解。

        std::ifstream nameFile(pmbusPath); // 開啟 sysfs name 檔，讀取 kernel driver/hwmon 名稱。
        if (!nameFile.good()) // 條件判斷，決定是否進入此分支。
        { // 程式區塊或初始化列表的結構符號。
            lg2::error("Failure finding '{PATH}'", "PATH", pmbusPath); // 記錄錯誤訊息。
            continue; // 跳過目前迴圈項目，處理下一個。
        } // 程式區塊或初始化列表的結構符號。

        std::string pmbusName; // 程式結構/變數設定，配合上下文理解。
        std::getline(nameFile, pmbusName); // 讀取 name 檔內容作為裝置名稱。
        nameFile.close(); // 程式結構/變數設定，配合上下文理解。

        if (std::ranges::find_if(sensorTypes.begin(), sensorTypes.end(), // PSU sensor 支援的 EntityManager 型別與 kernel driver 名稱白名單。
                                 [pmbusName](const auto& a) { // 程式結構/變數設定，配合上下文理解。
                                     return a.first == pmbusName; // 結束目前函式或提前離開。
                                 }) == sensorTypes.end()) // PSU sensor 支援的 EntityManager 型別與 kernel driver 名稱白名單。
        { // 程式區塊或初始化列表的結構符號。
            // To avoid this error message, add your driver name to // 原始碼內既有說明註解。
            // the pmbusNames vector at the top of this file. // 原始碼內既有說明註解。
            lg2::error("'{NAME}' not found in sensor whitelist", "NAME", // 若 driver 名稱不在 sensorTypes 白名單，則不建立 sensor。
                       pmbusName); // 程式結構/變數設定，配合上下文理解。
            continue; // 跳過目前迴圈項目，處理下一個。
        } // 程式區塊或初始化列表的結構符號。

        auto directory = pmbusPath.parent_path(); // 程式結構/變數設定，配合上下文理解。

        auto ret = directories.insert(directory.string()); // 程式結構/變數設定，配合上下文理解。
        if (!ret.second) // 條件判斷，決定是否進入此分支。
        { // 程式區塊或初始化列表的結構符號。
            lg2::error("Duplicate path: '{PATH}'", "PATH", directory); // 記錄錯誤訊息。
            continue; // check if path has already been searched // 跳過目前迴圈項目，處理下一個。
        } // 程式區塊或初始化列表的結構符號。

        DevTypes devType = DevTypes::HWMON; // sensor 來源類型列舉值。
        std::string deviceName; // 程式結構/變數設定，配合上下文理解。
        if (directory.parent_path() == "/sys/class/hwmon") // sensor 來源類型列舉值。
        { // 程式區塊或初始化列表的結構符號。
            std::string devicePath = // 程式結構/變數設定，配合上下文理解。
                std::filesystem::canonical(directory / "device"); // 解析 symlink，取得實際 sysfs 裝置路徑。
            std::smatch match; // 程式結構/變數設定，配合上下文理解。
            // Find /i2c-<bus>/<bus>-<address> match in device path // 原始碼內既有說明註解。
            std::regex_search(devicePath, match, devRegex); // 定義 regex，用來從 sysfs device path 擷取 I2C bus/address。
            if (match.empty()) // 條件判斷，決定是否進入此分支。
            { // 程式區塊或初始化列表的結構符號。
                lg2::error("Found bad device path: '{PATH}'", "PATH", // 記錄錯誤訊息。
                           devicePath); // 程式結構/變數設定，配合上下文理解。
                continue; // 跳過目前迴圈項目，處理下一個。
            } // 程式區塊或初始化列表的結構符號。
            // Extract <bus>-<address> // 原始碼內既有說明註解。
            std::string matchStr = match[1]; // 程式結構/變數設定，配合上下文理解。
            deviceName = matchStr.substr(matchStr.find_last_of('/') + 1); // 程式結構/變數設定，配合上下文理解。
        } // 程式區塊或初始化列表的結構符號。
        else // 前面條件不成立時執行。
        { // 程式區塊或初始化列表的結構符號。
            deviceName = // 程式結構/變數設定，配合上下文理解。
                std::filesystem::canonical(directory).parent_path().stem(); // 解析 symlink，取得實際 sysfs 裝置路徑。
            devType = DevTypes::IIO; // sensor 來源類型列舉值。
        } // 程式區塊或初始化列表的結構符號。

        size_t bus = 0; // 程式結構/變數設定，配合上下文理解。
        size_t addr = 0; // 程式結構/變數設定，配合上下文理解。
        if (!getDeviceBusAddr(deviceName, bus, addr)) // 從 device name 解析 I2C bus 與 address。
        { // 程式區塊或初始化列表的結構符號。
            continue; // 跳過目前迴圈項目，處理下一個。
        } // 程式區塊或初始化列表的結構符號。

        const SensorBaseConfigMap* baseConfig = nullptr; // 目前匹配到的 PSU 基本設定資料。
        const SensorData* sensorData = nullptr; // 程式結構/變數設定，配合上下文理解。
        const std::string* interfacePath = nullptr; // EntityManager config object path，用來關聯 sensor 與設定。
        std::string sensorType; // 程式結構/變數設定，配合上下文理解。
        size_t thresholdConfSize = 0; // 程式結構/變數設定，配合上下文理解。

        for (const auto& [path, cfgData] : sensorConfigs) // EntityManager 回傳的 sensor 設定集合。
        { // 程式區塊或初始化列表的結構符號。
            sensorData = &cfgData; // 程式結構/變數設定，配合上下文理解。
            for (const auto& [type, dt] : sensorTypes) // PSU sensor 支援的 EntityManager 型別與 kernel driver 名稱白名單。
            { // 程式區塊或初始化列表的結構符號。
                auto sensorBase = sensorData->find(configInterfaceName(type)); // 把 sensor type 轉成 EntityManager config interface 名稱。
                if (sensorBase != sensorData->end()) // 條件判斷，決定是否進入此分支。
                { // 程式區塊或初始化列表的結構符號。
                    baseConfig = &sensorBase->second; // 目前匹配到的 PSU 基本設定資料。
                    sensorType = type; // 程式結構/變數設定，配合上下文理解。
                    break; // 跳出目前迴圈。
                } // 程式區塊或初始化列表的結構符號。
            } // 程式區塊或初始化列表的結構符號。
            if (baseConfig == nullptr) // 目前匹配到的 PSU 基本設定資料。
            { // 程式區塊或初始化列表的結構符號。
                lg2::error("error finding base configuration for '{NAME}'", // 記錄錯誤訊息。
                           "NAME", deviceName); // 程式結構/變數設定，配合上下文理解。
                continue; // 跳過目前迴圈項目，處理下一個。
            } // 程式區塊或初始化列表的結構符號。

            auto configBus = baseConfig->find("Bus"); // 目前匹配到的 PSU 基本設定資料。
            auto configAddress = baseConfig->find("Address"); // 目前匹配到的 PSU 基本設定資料。

            if (configBus == baseConfig->end() || // 目前匹配到的 PSU 基本設定資料。
                configAddress == baseConfig->end()) // 目前匹配到的 PSU 基本設定資料。
            { // 程式區塊或初始化列表的結構符號。
                lg2::error("error finding necessary entry in configuration"); // 記錄錯誤訊息。
                continue; // 跳過目前迴圈項目，處理下一個。
            } // 程式區塊或初始化列表的結構符號。

            const uint64_t* confBus = // 設定檔裡的 bus/address 數值。
                std::get_if<uint64_t>(&(configBus->second)); // 程式結構/變數設定，配合上下文理解。
            const uint64_t* confAddr = // 設定檔裡的 bus/address 數值。
                std::get_if<uint64_t>(&(configAddress->second)); // 程式結構/變數設定，配合上下文理解。
            if (confBus == nullptr || confAddr == nullptr) // 設定檔裡的 bus/address 數值。
            { // 程式區塊或初始化列表的結構符號。
                lg2::error("Cannot get bus or address, invalid configuration"); // 記錄錯誤訊息。
                continue; // 跳過目前迴圈項目，處理下一個。
            } // 程式區塊或初始化列表的結構符號。

            if ((*confBus != bus) || (*confAddr != addr)) // 設定檔裡的 bus/address 數值。
            { // 程式區塊或初始化列表的結構符號。
                lg2::debug( // 記錄 debug 訊息。
                    "Configuration skipping '{CONFBUS}'-'{CONFADDR}' because not {BUS}-{ADDR}", // 設定檔裡的 bus/address 數值。
                    "CONFBUS", *confBus, "CONFADDR", *confAddr, "BUS", bus, // 設定檔裡的 bus/address 數值。
                    "ADDR", addr); // 程式結構/變數設定，配合上下文理解。
                continue; // 跳過目前迴圈項目，處理下一個。
            } // 程式區塊或初始化列表的結構符號。

            std::vector<thresholds::Threshold> confThresholds; // 程式結構/變數設定，配合上下文理解。
            if (!parseThresholdsFromConfig(*sensorData, confThresholds)) // 從設定檔解析 threshold 設定。
            { // 程式區塊或初始化列表的結構符號。
                lg2::error("error populating total thresholds"); // 記錄錯誤訊息。
            } // 程式區塊或初始化列表的結構符號。
            thresholdConfSize = confThresholds.size(); // 程式結構/變數設定，配合上下文理解。

            interfacePath = &path.str; // EntityManager config object path，用來關聯 sensor 與設定。
            break; // 跳出目前迴圈。
        } // 程式區塊或初始化列表的結構符號。
        if (interfacePath == nullptr) // EntityManager config object path，用來關聯 sensor 與設定。
        { // 程式區塊或初始化列表的結構符號。
            // To avoid this error message, add your export map entry, // 原始碼內既有說明註解。
            // from Entity Manager, to sensorTypes at the top of this file. // 原始碼內既有說明註解。
            lg2::error("failed to find match for '{NAME}'", "NAME", deviceName); // 記錄錯誤訊息。
            continue; // 跳過目前迴圈項目，處理下一個。
        } // 程式區塊或初始化列表的結構符號。

        auto findI2CDev = devices.find(*interfacePath); // EntityManager config object path，用來關聯 sensor 與設定。

        std::shared_ptr<I2CDevice> i2cDev; // 尋找或保存對應的 I2C device 管理物件。
        if (findI2CDev != devices.end()) // 尋找或保存對應的 I2C device 管理物件。
        { // 程式區塊或初始化列表的結構符號。
            if (activateOnly && !findI2CDev->second.second) // 尋找或保存對應的 I2C device 管理物件。
            { // 程式區塊或初始化列表的結構符號。
                continue; // 跳過目前迴圈項目，處理下一個。
            } // 程式區塊或初始化列表的結構符號。
            i2cDev = findI2CDev->second.first; // 尋找或保存對應的 I2C device 管理物件。
        } // 程式區塊或初始化列表的結構符號。

        auto findPSUName = baseConfig->find("Name"); // 目前匹配到的 PSU 基本設定資料。
        if (findPSUName == baseConfig->end()) // 目前匹配到的 PSU 基本設定資料。
        { // 程式區塊或初始化列表的結構符號。
            lg2::error("could not determine configuration name for '{NAME}'", // 記錄錯誤訊息。
                       "NAME", deviceName); // 程式結構/變數設定，配合上下文理解。
            continue; // 跳過目前迴圈項目，處理下一個。
        } // 程式區塊或初始化列表的結構符號。
        const std::string* psuName = // 程式結構/變數設定，配合上下文理解。
            std::get_if<std::string>(&(findPSUName->second)); // 程式結構/變數設定，配合上下文理解。
        if (psuName == nullptr) // 條件判斷，決定是否進入此分支。
        { // 程式區塊或初始化列表的結構符號。
            lg2::error("Cannot find psu name, invalid configuration"); // 記錄錯誤訊息。
            continue; // 跳過目前迴圈項目，處理下一個。
        } // 程式區塊或初始化列表的結構符號。

        auto findCPU = baseConfig->find("CPURequired"); // 目前匹配到的 PSU 基本設定資料。
        if (findCPU != baseConfig->end()) // 目前匹配到的 PSU 基本設定資料。
        { // 程式區塊或初始化列表的結構符號。
            size_t index = std::visit(VariantToIntVisitor(), findCPU->second); // 把 variant 設定值轉成整數。
            auto presenceFind = cpuPresence.find(index); // 記錄 CPU 是否存在，用來決定某些 sensor 是否要建立。
            if (presenceFind == cpuPresence.end() || !presenceFind->second) // 記錄 CPU 是否存在，用來決定某些 sensor 是否要建立。
            { // 程式區塊或初始化列表的結構符號。
                continue; // 跳過目前迴圈項目，處理下一個。
            } // 程式區塊或初始化列表的結構符號。
        } // 程式區塊或初始化列表的結構符號。

        // on rescans, only update sensors we were signaled by // 原始碼內既有說明註解。
        if (!firstScan) // 條件判斷，決定是否進入此分支。
        { // 程式區塊或初始化列表的結構符號。
            std::string psuNameStr = "/" + escapeName(*psuName); // 程式結構/變數設定，配合上下文理解。
            auto it = // 程式結構/變數設定，配合上下文理解。
                std::find_if(sensorsChanged->begin(), sensorsChanged->end(), // 處理 EntityManager 設定變更時，只重建受影響的 sensor。
                             [psuNameStr](std::string& s) { // 程式結構/變數設定，配合上下文理解。
                                 return s.ends_with(psuNameStr); // 結束目前函式或提前離開。
                             }); // 程式區塊或初始化列表的結構符號。

            if (it == sensorsChanged->end()) // 處理 EntityManager 設定變更時，只重建受影響的 sensor。
            { // 程式區塊或初始化列表的結構符號。
                continue; // 跳過目前迴圈項目，處理下一個。
            } // 程式區塊或初始化列表的結構符號。
            sensorsChanged->erase(it); // 處理 EntityManager 設定變更時，只重建受影響的 sensor。
        } // 程式區塊或初始化列表的結構符號。
        checkEvent(directory.string(), eventMatch, eventPathList); // 定義 PSU 狀態事件與 sysfs alarm/beep 檔案的對應。
        checkGroupEvent(directory.string(), groupEventPathList); // 收集 PSU 群組事件檔案。

        PowerState readState = getPowerState(*baseConfig); // 目前匹配到的 PSU 基本設定資料。

        /* Check if there are more sensors in the same interface */ // 授權或檔頭註解開始。
        int i = 1; // 授權或檔頭註解內容。
        std::vector<std::string> psuNames; // 授權或檔頭註解內容。
        do // 授權或檔頭註解內容。
        { // 授權或檔頭註解內容。
            // Individual string fields: Name, Name1, Name2, Name3, ... // 授權或檔頭註解內容。
            psuNames.push_back( // 授權或檔頭註解內容。
                escapeName(std::get<std::string>(findPSUName->second))); // 授權或檔頭註解內容。
            findPSUName = baseConfig->find("Name" + std::to_string(i++)); // 授權或檔頭註解內容。
        } while (findPSUName != baseConfig->end()); // 授權或檔頭註解內容。

        std::vector<std::filesystem::path> sensorPaths; // 授權或檔頭註解內容。
        const auto* param = std::find_if( // 授權或檔頭註解內容。
            devParamMap.begin(), devParamMap.end(), // 授權或檔頭註解內容。
            [devType](const auto& p) { return p.first == devType; }); // 授權或檔頭註解內容。
        if (param == devParamMap.end()) // 授權或檔頭註解內容。
        { // 授權或檔頭註解內容。
            lg2::error("No dev param map found for dev type: {TYPE}", "TYPE", // 授權或檔頭註解內容。
                       devType); // 授權或檔頭註解內容。
            continue; // 授權或檔頭註解內容。
        } // 授權或檔頭註解內容。

        if (!findFiles(directory, param->second.matchRegEx, sensorPaths, 0)) // 授權或檔頭註解內容。
        { // 授權或檔頭註解內容。
            lg2::error("No PSU non-label sensor in PSU"); // 授權或檔頭註解內容。
            continue; // 授權或檔頭註解內容。
        } // 授權或檔頭註解內容。

        /* read max value in sysfs for in, curr, power, temp, ... */ // 授權或檔頭註解開始。
        if (!findFiles(directory, R"(\w\d+_max$)", sensorPaths, 0)) // 授權或檔頭註解內容。
        { // 授權或檔頭註解內容。
            lg2::debug("No max name in PSU"); // 授權或檔頭註解內容。
        } // 授權或檔頭註解內容。

        float pollRate = getPollRate(*baseConfig, PSUSensor::defaultSensorPoll); // 授權或檔頭註解內容。

        /* Find array of labels to be exposed if it is defined in config */ // 授權或檔頭註解開始。
        std::vector<std::string> findLabels; // 授權或檔頭註解內容。
        auto findLabelObj = baseConfig->find("Labels"); // 授權或檔頭註解內容。
        if (findLabelObj != baseConfig->end()) // 授權或檔頭註解內容。
        { // 授權或檔頭註解內容。
            findLabels = // 授權或檔頭註解內容。
                std::get<std::vector<std::string>>(findLabelObj->second); // 授權或檔頭註解內容。
        } // 授權或檔頭註解內容。
        const auto* devParam = std::find_if( // 授權或檔頭註解內容。
            devParamMap.begin(), devParamMap.end(), // 授權或檔頭註解內容。
            [devType](const auto& p) { return p.first == devType; }); // 授權或檔頭註解內容。
        if (devParam == devParamMap.end()) // 授權或檔頭註解內容。
        { // 授權或檔頭註解內容。
            lg2::error("No dev param map found for dev type: {TYPE}", "TYPE", // 授權或檔頭註解內容。
                       devType); // 授權或檔頭註解內容。
            continue; // 授權或檔頭註解內容。
        } // 授權或檔頭註解內容。

        std::regex sensorNameRegEx(std::string(devParam->second.nameRegEx)); // 授權或檔頭註解內容。
        std::smatch matches; // 授權或檔頭註解內容。

        for (const auto& sensorPath : sensorPaths) // 授權或檔頭註解內容。
        { // 授權或檔頭註解內容。
            bool maxLabel = false; // 授權或檔頭註解內容。
            std::string labelHead; // 授權或檔頭註解內容。
            std::string sensorPathStr = sensorPath.string(); // 授權或檔頭註解內容。
            std::string sensorNameStr = sensorPath.filename(); // 授權或檔頭註解內容。
            std::string sensorNameSubStr; // 授權或檔頭註解內容。
            if (std::regex_search(sensorNameStr, matches, sensorNameRegEx)) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                // hwmon *_input filename without number: // 授權或檔頭註解內容。
                // in, curr, power, temp, ... // 授權或檔頭註解內容。
                // iio in_*_raw filename without number: // 授權或檔頭註解內容。
                // voltage, temp, pressure, ... // 授權或檔頭註解內容。

                const auto* param = std::find_if( // 授權或檔頭註解內容。
                    devParamMap.begin(), devParamMap.end(), // 授權或檔頭註解內容。
                    [devType](const auto& p) { return p.first == devType; }); // 授權或檔頭註解內容。
                if (param == devParamMap.end()) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    lg2::error("No dev param map found for dev type: {TYPE}", // 授權或檔頭註解內容。
                               "TYPE", devType); // 授權或檔頭註解內容。
                    continue; // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。

                sensorNameSubStr = matches[param->second.matchIndex]; // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。
            else // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                lg2::error("Could not extract the alpha prefix from '{NAME}'", // 授權或檔頭註解內容。
                           "NAME", sensorNameStr); // 授權或檔頭註解內容。
                continue; // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。

            std::string labelPath; // 授權或檔頭註解內容。

            if (devType == DevTypes::HWMON) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                /* find and differentiate _max and _input to replace "label" */ // 授權或檔頭註解開始。
                size_t pos = sensorPathStr.find('_'); // 授權或檔頭註解內容。
                if (pos != std::string::npos) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    std::string sensorPathStrMax = sensorPathStr.substr(pos); // 授權或檔頭註解內容。
                    if (sensorPathStrMax == "_max") // 授權或檔頭註解內容。
                    { // 授權或檔頭註解內容。
                        labelPath = boost::replace_all_copy(sensorPathStr, // 授權或檔頭註解內容。
                                                            "max", "label"); // 授權或檔頭註解內容。
                        maxLabel = true; // 授權或檔頭註解內容。
                    } // 授權或檔頭註解內容。
                    else // 授權或檔頭註解內容。
                    { // 授權或檔頭註解內容。
                        labelPath = boost::replace_all_copy(sensorPathStr, // 授權或檔頭註解內容。
                                                            "input", "label"); // 授權或檔頭註解內容。
                        maxLabel = false; // 授權或檔頭註解內容。
                    } // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
                else // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    continue; // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。

                std::ifstream labelFile(labelPath); // 授權或檔頭註解內容。
                if (!labelFile.good()) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    lg2::debug( // 授權或檔頭註解內容。
                        "Input file '{PATH}' has no corresponding label file", // 授權或檔頭註解內容。
                        "PATH", sensorPath.string()); // 授權或檔頭註解內容。
                    // hwmon *_input filename with number: // 授權或檔頭註解內容。
                    // temp1, temp2, temp3, ... // 授權或檔頭註解內容。
                    labelHead = // 授權或檔頭註解內容。
                        sensorNameStr.substr(0, sensorNameStr.find('_')); // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
                else // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    std::string label; // 授權或檔頭註解內容。
                    std::getline(labelFile, label); // 授權或檔頭註解內容。
                    labelFile.close(); // 授權或檔頭註解內容。
                    auto findSensor = sensors.find(label); // 授權或檔頭註解內容。
                    if (findSensor != sensors.end()) // 授權或檔頭註解內容。
                    { // 授權或檔頭註解內容。
                        continue; // 授權或檔頭註解內容。
                    } // 授權或檔頭註解內容。

                    // hwmon corresponding *_label file contents: // 授權或檔頭註解內容。
                    // vin1, vout1, ... // 授權或檔頭註解內容。
                    labelHead = label.substr(0, label.find(' ')); // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。

                /* append "max" for labelMatch */ // 授權或檔頭註解開始。
                if (maxLabel) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    labelHead.insert(0, "max"); // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。

                // Don't add PWM sensors if it's not in label list // 授權或檔頭註解內容。
                if (!findLabels.empty()) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    /* Check if this labelHead is enabled in config file */ // 授權或檔頭註解開始。
                    if (std::find(findLabels.begin(), findLabels.end(), // 授權或檔頭註解內容。
                                  labelHead) == findLabels.end()) // 授權或檔頭註解內容。
                    { // 授權或檔頭註解內容。
                        lg2::debug("could not find {LABEL} in the Labels list", // 授權或檔頭註解內容。
                                   "LABEL", labelHead); // 授權或檔頭註解內容。
                        continue; // 授權或檔頭註解內容。
                    } // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
                checkPWMSensor(sensorPath, labelHead, *interfacePath, // 授權或檔頭註解內容。
                               dbusConnection, objectServer, psuNames[0]); // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。
            else if (devType == DevTypes::IIO) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                auto findIIOHyphen = sensorNameStr.find_last_of('_'); // 授權或檔頭註解內容。
                labelHead = sensorNameStr.substr(0, findIIOHyphen); // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。

            lg2::debug("Sensor type: {NAME}, label: {LABEL}", "NAME", // 授權或檔頭註解內容。
                       sensorNameSubStr, "LABEL", labelHead); // 授權或檔頭註解內容。

            if (!findLabels.empty()) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                /* Check if this labelHead is enabled in config file */ // 授權或檔頭註解開始。
                if (std::find(findLabels.begin(), findLabels.end(), // 授權或檔頭註解內容。
                              labelHead) == findLabels.end()) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    lg2::debug("could not find '{LABEL}' in the Labels list", // 授權或檔頭註解內容。
                               "LABEL", labelHead); // 授權或檔頭註解內容。
                    continue; // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。
            auto it = std::find_if(labelHead.begin(), labelHead.end(), // 授權或檔頭註解內容。
                                   static_cast<int (*)(int)>(std::isdigit)); // 授權或檔頭註解內容。
            std::string_view labelHeadView( // 授權或檔頭註解內容。
                labelHead.data(), std::distance(labelHead.begin(), it)); // 授權或檔頭註解內容。
            const auto* findProperty = std::ranges::find_if( // 授權或檔頭註解內容。
                labelMatch, [&labelHeadView](const auto& a) { // 授權或檔頭註解內容。
                    return a.hwmonLabelName == labelHeadView; // 授權或檔頭註解內容。
                }); // 授權或檔頭註解內容。
            if (findProperty == labelMatch.end()) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                lg2::debug( // 授權或檔頭註解內容。
                    "Could not find matching default property for '{LABEL}'", // 授權或檔頭註解內容。
                    "LABEL", labelHead); // 授權或檔頭註解內容。
                continue; // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。

            // Protect the hardcoded labelMatch list from changes, // 授權或檔頭註解內容。
            // by making a copy and modifying that instead. // 授權或檔頭註解內容。
            // Avoid bleedthrough of one device's customizations to // 授權或檔頭註解內容。
            // the next device, as each should be independently customizable. // 授權或檔頭註解內容。
            const PSUProperty& psuProperty = *findProperty; // 授權或檔頭註解內容。
            std::string labelTypeName(psuProperty.labelTypeName); // 授權或檔頭註解內容。
            double sensorScaleFactor = psuProperty.sensorScaleFactor; // 授權或檔頭註解內容。
            double maxReading = psuProperty.maxReading; // 授權或檔頭註解內容。
            double minReading = psuProperty.minReading; // 授權或檔頭註解內容。
            double sensorOffset = psuProperty.sensorOffset; // 授權或檔頭註解內容。

            // Use label head as prefix for reading from config file, // 授權或檔頭註解內容。
            // example if temp1: temp1_Name, temp1_Scale, temp1_Min, ... // 授權或檔頭註解內容。
            std::string keyName = labelHead + "_Name"; // 授權或檔頭註解內容。
            std::string keyScale = labelHead + "_Scale"; // 授權或檔頭註解內容。
            std::string keyMin = labelHead + "_Min"; // 授權或檔頭註解內容。
            std::string keyMax = labelHead + "_Max"; // 授權或檔頭註解內容。
            std::string keyOffset = labelHead + "_Offset"; // 授權或檔頭註解內容。
            std::string keyPowerState = labelHead + "_PowerState"; // 授權或檔頭註解內容。

            bool customizedName = false; // 授權或檔頭註解內容。
            auto findCustomName = baseConfig->find(keyName); // 授權或檔頭註解內容。

            if (findCustomName != baseConfig->end()) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                try // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    labelTypeName = std::visit(VariantToStringVisitor(), // 授權或檔頭註解內容。
                                               findCustomName->second); // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
                catch (const std::invalid_argument&) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    lg2::error("Unable to parse '{NAME}'", "NAME", keyName); // 授權或檔頭註解內容。
                    continue; // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。

                // All strings are valid, including empty string // 授權或檔頭註解內容。
                customizedName = true; // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。

            bool customizedScale = false; // 授權或檔頭註解內容。
            auto findCustomScale = baseConfig->find(keyScale); // 授權或檔頭註解內容。
            if (findCustomScale != baseConfig->end()) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                try // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    sensorScaleFactor = std::visit( // 授權或檔頭註解內容。
                        VariantToUnsignedIntVisitor(), findCustomScale->second); // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
                catch (const std::invalid_argument&) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    lg2::error("Unable to parse '{SCALE}'", "SCALE", keyScale); // 授權或檔頭註解內容。
                    continue; // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。

                // Avoid later division by zero // 授權或檔頭註解內容。
                if (sensorScaleFactor > 0) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    customizedScale = true; // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
                else // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    lg2::error("Unable to accept '{SCALE}'", "SCALE", keyScale); // 授權或檔頭註解內容。
                    continue; // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。

            auto findCustomMin = baseConfig->find(keyMin); // 授權或檔頭註解內容。
            if (findCustomMin != baseConfig->end()) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                try // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    minReading = std::visit(VariantToDoubleVisitor(), // 授權或檔頭註解內容。
                                            findCustomMin->second); // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
                catch (const std::invalid_argument&) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    lg2::error("Unable to parse '{MIN}'", "MIN", keyMin); // 授權或檔頭註解內容。
                    continue; // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。

            auto findCustomMax = baseConfig->find(keyMax); // 授權或檔頭註解內容。
            if (findCustomMax != baseConfig->end()) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                try // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    maxReading = std::visit(VariantToDoubleVisitor(), // 授權或檔頭註解內容。
                                            findCustomMax->second); // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
                catch (const std::invalid_argument&) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    lg2::error("Unable to parse '{MAX}'", "MAX", keyMax); // 授權或檔頭註解內容。
                    continue; // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。

            auto findCustomOffset = baseConfig->find(keyOffset); // 授權或檔頭註解內容。
            if (findCustomOffset != baseConfig->end()) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                try // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    sensorOffset = std::visit(VariantToDoubleVisitor(), // 授權或檔頭註解內容。
                                              findCustomOffset->second); // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
                catch (const std::invalid_argument&) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    lg2::error("Unable to parse '{OFFSET}'", "OFFSET", // 授權或檔頭註解內容。
                               keyOffset); // 授權或檔頭註解內容。
                    continue; // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。

            // if we find label head power state set ，override the powerstate. // 授權或檔頭註解內容。
            auto findPowerState = baseConfig->find(keyPowerState); // 授權或檔頭註解內容。
            if (findPowerState != baseConfig->end()) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                std::string powerState = std::visit(VariantToStringVisitor(), // 授權或檔頭註解內容。
                                                    findPowerState->second); // 授權或檔頭註解內容。
                setReadState(powerState, readState); // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。
            if (!(minReading < maxReading)) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                lg2::error("Min must be less than Max"); // 授權或檔頭註解內容。
                continue; // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。

            // If the sensor name is being customized by config file, // 授權或檔頭註解內容。
            // then prefix/suffix composition becomes not necessary, // 授權或檔頭註解內容。
            // and in fact not wanted, because it gets in the way. // 授權或檔頭註解內容。
            std::string psuNameFromIndex; // 授權或檔頭註解內容。
            std::string nameIndexStr = "1"; // 授權或檔頭註解內容。
            if (!customizedName) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                /* Find out sensor name index for this label */ // 授權或檔頭註解開始。
                std::regex rgx("[A-Za-z]+([0-9]+)"); // 授權或檔頭註解內容。
                size_t nameIndex{0}; // 授權或檔頭註解內容。
                if (std::regex_search(labelHead, matches, rgx)) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    nameIndexStr = matches[1]; // 授權或檔頭註解內容。
                    nameIndex = std::stoi(nameIndexStr); // 授權或檔頭註解內容。

                    // Decrement to preserve alignment, because hwmon // 授權或檔頭註解內容。
                    // human-readable filenames and labels use 1-based // 授權或檔頭註解內容。
                    // numbering, but the "Name", "Name1", "Name2", etc. naming // 授權或檔頭註解內容。
                    // convention (the psuNames vector) uses 0-based numbering. // 授權或檔頭註解內容。
                    if (nameIndex > 0) // 授權或檔頭註解內容。
                    { // 授權或檔頭註解內容。
                        --nameIndex; // 授權或檔頭註解內容。
                    } // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
                else // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    nameIndex = 0; // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。

                if (psuNames.size() <= nameIndex) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    lg2::error("Could not pair '{LABEL}' with a Name field", // 授權或檔頭註解內容。
                               "LABEL", labelHead); // 授權或檔頭註解內容。
                    continue; // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。

                psuNameFromIndex = psuNames[nameIndex]; // 授權或檔頭註解內容。

                lg2::debug("'{LABEL}' paired with '{NAME}' at index '{INDEX}'", // 授權或檔頭註解內容。
                           "LABEL", labelHead, "NAME", psuNameFromIndex, // 授權或檔頭註解內容。
                           "INDEX", nameIndex); // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。

            if (devType == DevTypes::HWMON) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                checkEventLimits(sensorPathStr, limitEventMatch, eventPathList); // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。

            // Similarly, if sensor scaling factor is being customized, // 授權或檔頭註解內容。
            // then the below power-of-10 constraint becomes unnecessary, // 授權或檔頭註解內容。
            // as config should be able to specify an arbitrary divisor. // 授權或檔頭註解內容。
            unsigned int factor = sensorScaleFactor; // 授權或檔頭註解內容。
            if (!customizedScale) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                // Preserve existing usage of hardcoded labelMatch table below // 授權或檔頭註解內容。
                factor = std::pow(10.0, factor); // 授權或檔頭註解內容。

                /* Change first char of substring to uppercase */ // 授權或檔頭註解開始。
                char firstChar = // 授權或檔頭註解內容。
                    static_cast<char>(std::toupper(sensorNameSubStr[0])); // 授權或檔頭註解內容。
                std::string strScaleFactor = // 授權或檔頭註解內容。
                    firstChar + sensorNameSubStr.substr(1) + "ScaleFactor"; // 授權或檔頭註解內容。

                // Preserve existing configs by accepting earlier syntax, // 授權或檔頭註解內容。
                // example CurrScaleFactor, PowerScaleFactor, ... // 授權或檔頭註解內容。
                auto findScaleFactor = baseConfig->find(strScaleFactor); // 授權或檔頭註解內容。
                if (findScaleFactor != baseConfig->end()) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    factor = std::visit(VariantToIntVisitor(), // 授權或檔頭註解內容。
                                        findScaleFactor->second); // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。

                lg2::debug( // 授權或檔頭註解內容。
                    "Sensor scaling factor '{FACTOR}' string '{SCALE_FACTOR}'", // 授權或檔頭註解內容。
                    "FACTOR", factor, "SCALE_FACTOR", strScaleFactor); // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。

            std::vector<thresholds::Threshold> sensorThresholds; // 授權或檔頭註解內容。
            if (!parseThresholdsFromConfig(*sensorData, sensorThresholds, // 授權或檔頭註解內容。
                                           &labelHead)) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                lg2::error("error populating thresholds for '{NAME}'", "NAME", // 授權或檔頭註解內容。
                           sensorNameSubStr); // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。

            const auto* findSensorUnit = std::ranges::find_if( // 授權或檔頭註解內容。
                sensorTable, [&sensorNameSubStr](const SensorUnit& a) { // 授權或檔頭註解內容。
                    return a.name == sensorNameSubStr; // 授權或檔頭註解內容。
                }); // 授權或檔頭註解內容。
            if (findSensorUnit == sensorTable.end()) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                lg2::error("'{NAME}' is not a recognized sensor type", "NAME", // 授權或檔頭註解內容。
                           sensorNameSubStr); // 授權或檔頭註解內容。
                continue; // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。

            lg2::debug("Sensor properties - Name: {NAME}, Scale: {SCALE}, " // 授權或檔頭註解內容。
                       "Min: {MIN}, Max: {MAX}, Offset: {OFFSET}", // 授權或檔頭註解內容。
                       "NAME", psuProperty.labelTypeName, "SCALE", // 授權或檔頭註解內容。
                       psuProperty.sensorScaleFactor, "MIN", // 授權或檔頭註解內容。
                       psuProperty.minReading, "MAX", psuProperty.maxReading, // 授權或檔頭註解內容。
                       "OFFSET", psuProperty.sensorOffset); // 授權或檔頭註解內容。

            std::string sensorName(labelTypeName); // 授權或檔頭註解內容。
            if (customizedName) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                if (sensorName.empty()) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    // Allow selective disabling of an individual sensor, // 授權或檔頭註解內容。
                    // by customizing its name to an empty string. // 授權或檔頭註解內容。
                    lg2::error("Sensor disabled, empty string"); // 授權或檔頭註解內容。
                    continue; // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。
            else // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                // Sensor name not customized, do prefix/suffix composition, // 授權或檔頭註解內容。
                // preserving default behavior by using psuNameFromIndex. // 授權或檔頭註解內容。
                sensorName = // 授權或檔頭註解內容。
                    std::format("{} {}", psuNameFromIndex, labelTypeName); // 授權或檔頭註解內容。

                // The labelTypeName of a fan can be: // 授權或檔頭註解內容。
                // "Fan Speed 1", "Fan Speed 2", "Fan Speed 3" ... // 授權或檔頭註解內容。
                if (labelHead == "fan" + nameIndexStr) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    sensorName += nameIndexStr; // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。

            lg2::debug("Sensor name: {NAME}, path: {PATH}, type: {TYPE}", // 授權或檔頭註解內容。
                       "NAME", sensorName, "PATH", sensorPathStr, "TYPE", // 授權或檔頭註解內容。
                       sensorType); // 授權或檔頭註解內容。
            // destruct existing one first if already created // 授權或檔頭註解內容。

            auto& sensor = sensors[sensorName]; // 授權或檔頭註解內容。
            if (!activateOnly) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                sensor = nullptr; // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。

            if (sensor != nullptr) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                sensor->activate(sensorPathStr, i2cDev); // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。
            else // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                sensors[sensorName] = std::make_shared<PSUSensor>( // 授權或檔頭註解內容。
                    sensorPathStr, sensorType, objectServer, dbusConnection, io, // 授權或檔頭註解內容。
                    sensorName, std::move(sensorThresholds), *interfacePath, // 授權或檔頭註解內容。
                    readState, findSensorUnit->units, factor, maxReading, // 授權或檔頭註解內容。
                    minReading, sensorOffset, labelHead, thresholdConfSize, // 授權或檔頭註解內容。
                    pollRate, i2cDev); // 授權或檔頭註解內容。
                sensors[sensorName]->setupRead(); // 授權或檔頭註解內容。
                ++numCreated; // 授權或檔頭註解內容。
                lg2::debug("Created '{NUM}' sensors so far", "NUM", numCreated); // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。
        } // 授權或檔頭註解內容。

        if (devType == DevTypes::HWMON) // 授權或檔頭註解內容。
        { // 授權或檔頭註解內容。
            // OperationalStatus event // 授權或檔頭註解內容。
            combineEvents[*psuName + "OperationalStatus"] = nullptr; // 授權或檔頭註解內容。
            combineEvents[*psuName + "OperationalStatus"] = // 授權或檔頭註解內容。
                std::make_unique<PSUCombineEvent>( // 授權或檔頭註解內容。
                    objectServer, dbusConnection, io, *psuName, readState, // 授權或檔頭註解內容。
                    eventPathList, groupEventPathList, "OperationalStatus", // 授權或檔頭註解內容。
                    pollRate); // 授權或檔頭註解內容。
        } // 授權或檔頭註解內容。
    } // 授權或檔頭註解內容。

    lg2::debug("Created total of '{NUM}' sensors", "NUM", numCreated); // 授權或檔頭註解內容。
} // 授權或檔頭註解內容。

static void getPresentCpus( // 授權或檔頭註解內容。
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection) // 授權或檔頭註解內容。
{ // 授權或檔頭註解內容。
    static const int depth = 2; // 授權或檔頭註解內容。
    static const int numKeys = 1; // 授權或檔頭註解內容。
    GetSubTreeType cpuSubTree; // 授權或檔頭註解內容。

    try // 授權或檔頭註解內容。
    { // 授權或檔頭註解內容。
        auto getItems = dbusConnection->new_method_call( // 授權或檔頭註解內容。
            mapper::busName, mapper::path, mapper::interface, mapper::subtree); // 授權或檔頭註解內容。
        getItems.append(cpuInventoryPath, static_cast<int32_t>(depth), // 授權或檔頭註解內容。
                        std::array<const char*, numKeys>{ // 授權或檔頭註解內容。
                            "xyz.openbmc_project.Inventory.Item"}); // 授權或檔頭註解內容。
        auto getItemsResp = dbusConnection->call(getItems); // 授權或檔頭註解內容。
        getItemsResp.read(cpuSubTree); // 授權或檔頭註解內容。
    } // 授權或檔頭註解內容。
    catch (sdbusplus::exception_t& e) // 授權或檔頭註解內容。
    { // 授權或檔頭註解內容。
        lg2::error("error getting inventory item subtree: '{ERR}'", "ERR", e); // 授權或檔頭註解內容。
        return; // 授權或檔頭註解內容。
    } // 授權或檔頭註解內容。

    for (const auto& [path, objDict] : cpuSubTree) // 授權或檔頭註解內容。
    { // 授權或檔頭註解內容。
        auto obj = sdbusplus::message::object_path(path).filename(); // 授權或檔頭註解內容。
        boost::to_lower(obj); // 授權或檔頭註解內容。

        if (!obj.starts_with("cpu") || objDict.empty()) // 授權或檔頭註解內容。
        { // 授權或檔頭註解內容。
            continue; // 授權或檔頭註解內容。
        } // 授權或檔頭註解內容。
        const std::string& owner = objDict.begin()->first; // 授權或檔頭註解內容。

        std::variant<bool> respValue; // 授權或檔頭註解內容。
        try // 授權或檔頭註解內容。
        { // 授權或檔頭註解內容。
            auto getPresence = dbusConnection->new_method_call( // 授權或檔頭註解內容。
                owner.c_str(), path.c_str(), "org.freedesktop.DBus.Properties", // 授權或檔頭註解內容。
                "Get"); // 授權或檔頭註解內容。
            getPresence.append("xyz.openbmc_project.Inventory.Item", "Present"); // 授權或檔頭註解內容。
            auto resp = dbusConnection->call(getPresence); // 授權或檔頭註解內容。
            resp.read(respValue); // 授權或檔頭註解內容。
        } // 授權或檔頭註解內容。
        catch (sdbusplus::exception_t& e) // 授權或檔頭註解內容。
        { // 授權或檔頭註解內容。
            lg2::error("Error in getting CPU presence: '{ERR}'", "ERR", e); // 授權或檔頭註解內容。
            continue; // 授權或檔頭註解內容。
        } // 授權或檔頭註解內容。

        auto* present = std::get_if<bool>(&respValue); // 授權或檔頭註解內容。
        if (present != nullptr && *present) // 授權或檔頭註解內容。
        { // 授權或檔頭註解內容。
            int cpuIndex = 0; // 授權或檔頭註解內容。
            try // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                cpuIndex = std::stoi(obj.substr(obj.size() - 1)); // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。
            catch (const std::exception& e) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                lg2::error("Error converting CPU index: '{ERR}'", "ERR", e); // 授權或檔頭註解內容。
                continue; // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。
            cpuPresence[cpuIndex] = *present; // 授權或檔頭註解內容。
        } // 授權或檔頭註解內容。
    } // 授權或檔頭註解內容。
} // 授權或檔頭註解內容。

void createSensors( // 授權或檔頭註解內容。
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer, // 授權或檔頭註解內容。
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection, // 授權或檔頭註解內容。
    const std::shared_ptr<boost::container::flat_set<std::string>>& // 授權或檔頭註解內容。
        sensorsChanged, // 授權或檔頭註解內容。
    bool activateOnly) // 授權或檔頭註解內容。
{ // 授權或檔頭註解內容。
    auto getter = std::make_shared<GetSensorConfiguration>( // 授權或檔頭註解內容。
        dbusConnection, [&io, &objectServer, &dbusConnection, sensorsChanged, // 授權或檔頭註解內容。
                         activateOnly](const ManagedObjectType& sensorConfigs) { // 授權或檔頭註解內容。
            createSensorsCallback(io, objectServer, dbusConnection, // 授權或檔頭註解內容。
                                  sensorConfigs, sensorsChanged, activateOnly); // 授權或檔頭註解內容。
        }); // 授權或檔頭註解內容。
    std::vector<std::string_view> types; // 授權或檔頭註解內容。
    types.reserve(sensorTypes.size()); // 授權或檔頭註解內容。
    for (const auto& [type, dt] : sensorTypes) // 授權或檔頭註解內容。
    { // 授權或檔頭註解內容。
        types.emplace_back(type); // 授權或檔頭註解內容。
    } // 授權或檔頭註解內容。
    getter->getConfiguration(types); // 授權或檔頭註解內容。
} // 授權或檔頭註解內容。

static void powerStateChanged( // 授權或檔頭註解內容。
    PowerState type, bool newState, // 授權或檔頭註解內容。
    boost::container::flat_map<std::string, std::shared_ptr<PSUSensor>>& // 授權或檔頭註解內容。
        sensors, // 授權或檔頭註解內容。
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer, // 授權或檔頭註解內容。
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection) // 授權或檔頭註解內容。
{ // 授權或檔頭註解內容。
    if (newState) // 授權或檔頭註解內容。
    { // 授權或檔頭註解內容。
        createSensors(io, objectServer, dbusConnection, nullptr, true); // 授權或檔頭註解內容。
    } // 授權或檔頭註解內容。
    else // 授權或檔頭註解內容。
    { // 授權或檔頭註解內容。
        for (auto& [path, sensor] : sensors) // 授權或檔頭註解內容。
        { // 授權或檔頭註解內容。
            if (sensor != nullptr && sensor->readState == type) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                sensor->deactivate(); // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。
        } // 授權或檔頭註解內容。
    } // 授權或檔頭註解內容。
} // 授權或檔頭註解內容。

int main() // 授權或檔頭註解內容。
{ // 授權或檔頭註解內容。
    boost::asio::io_context io; // 授權或檔頭註解內容。
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io); // 授權或檔頭註解內容。

    sdbusplus::asio::object_server objectServer(systemBus, true); // 授權或檔頭註解內容。
    objectServer.add_manager("/xyz/openbmc_project/sensors"); // 授權或檔頭註解內容。
    objectServer.add_manager("/xyz/openbmc_project/control"); // 授權或檔頭註解內容。
    systemBus->request_name("xyz.openbmc_project.PSUSensor"); // 授權或檔頭註解內容。
    auto sensorsChanged = // 授權或檔頭註解內容。
        std::make_shared<boost::container::flat_set<std::string>>(); // 授權或檔頭註解內容。

    auto powerCallBack = [&io, &objectServer, // 授權或檔頭註解內容。
                          &systemBus](PowerState type, bool state) { // 授權或檔頭註解內容。
        powerStateChanged(type, state, sensors, io, objectServer, systemBus); // 授權或檔頭註解內容。
    }; // 授權或檔頭註解內容。

    setupPowerMatchCallback(systemBus, powerCallBack); // 授權或檔頭註解內容。

    boost::asio::post(io, [&]() { // 授權或檔頭註解內容。
        createSensors(io, objectServer, systemBus, nullptr, false); // 授權或檔頭註解內容。
    }); // 授權或檔頭註解內容。
    boost::asio::steady_timer filterTimer(io); // 授權或檔頭註解內容。
    std::function<void(sdbusplus::message_t&)> eventHandler = // 授權或檔頭註解內容。
        [&](sdbusplus::message_t& message) { // 授權或檔頭註解內容。
            sensorsChanged->insert(message.get_path()); // 授權或檔頭註解內容。
            filterTimer.expires_after(std::chrono::seconds(3)); // 授權或檔頭註解內容。
            filterTimer.async_wait([&](const boost::system::error_code& ec) { // 授權或檔頭註解內容。
                if (ec == boost::asio::error::operation_aborted) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    return; // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
                if (ec) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    lg2::error("timer error"); // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
                createSensors(io, objectServer, systemBus, sensorsChanged, // 授權或檔頭註解內容。
                              false); // 授權或檔頭註解內容。
            }); // 授權或檔頭註解內容。
        }; // 授權或檔頭註解內容。

    boost::asio::steady_timer cpuFilterTimer(io); // 授權或檔頭註解內容。
    std::function<void(sdbusplus::message_t&)> cpuPresenceHandler = // 授權或檔頭註解內容。
        [&](sdbusplus::message_t& message) { // 授權或檔頭註解內容。
            std::string path = message.get_path(); // 授權或檔頭註解內容。
            boost::to_lower(path); // 授權或檔頭註解內容。

            sdbusplus::message::object_path cpuPath(path); // 授權或檔頭註解內容。
            std::string cpuName = cpuPath.filename(); // 授權或檔頭註解內容。
            if (!cpuName.starts_with("cpu")) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                return; // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。
            size_t index = 0; // 授權或檔頭註解內容。
            try // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                index = std::stoi(path.substr(path.size() - 1)); // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。
            catch (const std::invalid_argument&) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                lg2::error("Found invalid path: '{PATH}'", "PATH", path); // 授權或檔頭註解內容。
                return; // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。

            std::string objectName; // 授權或檔頭註解內容。
            boost::container::flat_map<std::string, std::variant<bool>> values; // 授權或檔頭註解內容。
            message.read(objectName, values); // 授權或檔頭註解內容。
            auto findPresence = values.find("Present"); // 授權或檔頭註解內容。
            if (findPresence == values.end()) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                return; // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。
            try // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                cpuPresence[index] = std::get<bool>(findPresence->second); // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。
            catch (const std::bad_variant_access& err) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                return; // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。

            if (!cpuPresence[index]) // 授權或檔頭註解內容。
            { // 授權或檔頭註解內容。
                return; // 授權或檔頭註解內容。
            } // 授權或檔頭註解內容。
            cpuFilterTimer.expires_after(std::chrono::seconds(1)); // 授權或檔頭註解內容。
            cpuFilterTimer.async_wait([&](const boost::system::error_code& ec) { // 授權或檔頭註解內容。
                if (ec == boost::asio::error::operation_aborted) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    return; // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
                if (ec) // 授權或檔頭註解內容。
                { // 授權或檔頭註解內容。
                    lg2::error("timer error"); // 授權或檔頭註解內容。
                    return; // 授權或檔頭註解內容。
                } // 授權或檔頭註解內容。
                createSensors(io, objectServer, systemBus, nullptr, false); // 授權或檔頭註解內容。
            }); // 授權或檔頭註解內容。
        }; // 授權或檔頭註解內容。

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches = // 授權或檔頭註解內容。
        setupPropertiesChangedMatches(*systemBus, sensorTypes, eventHandler); // 授權或檔頭註解內容。

    matches.emplace_back(std::make_unique<sdbusplus::bus::match_t>( // 授權或檔頭註解內容。
        static_cast<sdbusplus::bus_t&>(*systemBus), // 授權或檔頭註解內容。
        "type='signal',member='PropertiesChanged',path_namespace='" + // 授權或檔頭註解內容。
            std::string(cpuInventoryPath) + // 授權或檔頭註解內容。
            "',arg0namespace='xyz.openbmc_project.Inventory.Item'", // 授權或檔頭註解內容。
        cpuPresenceHandler)); // 授權或檔頭註解內容。

    getPresentCpus(systemBus); // 授權或檔頭註解內容。

    setupManufacturingModeMatch(*systemBus); // 授權或檔頭註解內容。
    io.run(); // 授權或檔頭註解內容。
} // 授權或檔頭註解內容。
