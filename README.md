# Media-BLE
 蓝牙媒体

# Media-BLE-V1.0



本项目使用ESP32结合编码器做的一个蓝牙媒体控制器，主要是控制电脑的音频播放、音量调节及歌曲切换，支持QQ音乐、网易云音乐、网页播放器等。



基本功能：

- 单击 --- 播放/暂停
- 双击 --- 切换上一曲音乐
- 长按 --- 间隔1s切换下一曲音乐

上述切换过程中，都会闪绿光提示切换。



本项目使用Arduino对ESP32进行开发，所需安装的库如下：

Arduino:

​		--- BleKeyboard: https://gitee.com/fspace/ESP32-BLE-Keyboard

​		---  FastLED: https://github.com/FastLED/FastLED

​		--- OneButton: https://github.com/mathertel/OneButton



功能未完全完善，但是...



又不是不能用...



结构也没画，后续会选用更小封装的灯珠，增加灯珠数量，增加RGB灯光的效果，目前只有四个WS2812B的灯珠，5050封装的，比较大，浪费空间，另外打算增加锂电池进行供电，其他的慢慢做