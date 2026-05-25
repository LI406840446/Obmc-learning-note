##  io_context 是什麼？

```diff
io_context 是 Boost.Asio 提供的 C++ 物件：
!boost::asio::io_context
可以把它想成非同步事件迴圈 / event loop
OpenBMC 很多 daemon 都會用它來處理：
D-Bus 非同步 callback
timer
檔案 async read
socket
sensor polling
signal handling

它是 C++ library 的東西：
!#include <boost/asio/io_context.hpp>

@你可以把 io_context 想成一個「工作排程中心」
例如：
PSUSensor 說：
  每 1 秒幫我讀一次 /sys/class/hwmon/hwmonX/power1_input

PSUEvent 說：
  每 1 秒幫我讀一次 /sys/class/hwmon/hwmonX/power1_alarm

D-Bus 說：
  如果有 PropertiesChanged signal，幫我呼叫 callback

timer 說：
  3 秒後幫我重建 sensors
這些工作都丟給io_context
最後 main() 裡會呼叫：io.run();
```

##  非同步事件迴圈不是 process，也不是 thread。
```diff
三者關係:
process
  └── thread
        └── io_context event loop
              ├── D-Bus callback
              ├── timer callback
              ├── sysfs async read callback
              └── sensor polling callback
所以：
process：整個 psusensor 程式
thread：process 裡的執行緒
io_context：在 thread 裡跑的事件迴圈 / callback 排程器


!它會不會自己生 thread？
io_context 本身不一定會自己生 thread。

通常情況：
io.run();
只會在目前這個 thread 跑。

如果程式寫成這樣：
std::thread t1([&] { io.run(); });
std::thread t2([&] { io.run(); });
那同一個 io_context 就可以被多個 thread 執行，callback 可能在不同 thread 跑。

!它為什麼不用很多 thread？
因為 sensor daemon 的工作大多是：
等 timer
等 D-Bus signal
讀 sysfs 小檔案
更新 D-Bus property
這些不是大量 CPU 運算，而是很多「等待」。

如果用同步寫法會變成：
while (true)
{
    read sensor A;
    sleep(1);
    read sensor B;
    sleep(1);
    read sensor C;
    sleep(1);
}
會很卡、很難管理。

用 event loop 則是：
sensor A：1 秒後讀
sensor B：1 秒後讀
D-Bus：有 signal 再處理
timer：3 秒後重建 sensors
所有等待都交給 event loop 管理。
```
