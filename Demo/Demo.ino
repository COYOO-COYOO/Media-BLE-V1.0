/* 基本功能:
 *  1、单击：播放/暂停
 *  2、双击：切换上一曲音乐
 *  3、长按：1s切换一次下一曲音乐，直至按键松开
 *  4、右旋：增大音量
 *  5、左旋：减小音量
 *  6、其他:让我好好想想...
 */


/* 
 *  NimBLE模式可以显著节省RAM和FLASH存储器。
 */
 
#define USE_NIMBLE  // 激活NimBLE模式

#include <Arduino.h>
#include <BleKeyboard.h>
#include <WiFi.h>
#include <FastLED.h>
#include "OneButton.h"


#if defined(ESP32)
#define PIN_INPUT 14
#endif


OneButton button(PIN_INPUT, true);

// 其中“ESP32蓝牙键盘”为键盘名称；"Espressif"为制造商，电量为100
BleKeyboard bleKeyboard("Media BLE", "coyoo", 100); 


unsigned long pressStartTime;

#define Delay_mtime 10

#if defined(ESP32)
void IRAM_ATTR checkTicks() {
    button.tick();
}
#endif

// 单击状态   双击状态   长按状态
bool single_state = 0;
bool double_state = 0;
bool PressState = 0;


/* 单击暂停/播放 */
void singleClick() {
    Serial.println("singleClick() detected.");
    single_state = 1;
}

/* 双击播放上一曲 */
void doubleClick() {
    Serial.println("doubleClick() detected.");
    double_state = 1;
    
}

/* 多次按键按下 */
void multiClick() {
    int n = button.getNumberClicks();
    if (n == 3) {
        Serial.println("tripleClick detected.");
    } 
    else if (n == 4) {
        Serial.println("quadrupleClick detected.");
    } 
    else {
        Serial.print("multiClick(");
        Serial.print(n);
        Serial.println(") detected.");
    }
}


/* 长按---开始时间 */
void pressStart() {
    Serial.println("pressStart()::::");
    pressStartTime = millis() - 1000;
//    pressStartTime = 0;
    Serial.println(pressStartTime);
    PressState = 1;
}

/* 长按---结束时间 */
void pressStop() {
//    Serial.print("pressStop(");
//    Serial.print(millis() - pressStartTime);
//    Serial.println(") detected.");

    PressState = 0;
}


/**
  * @brief  编码器初始化
  * @param  Encoder_Count：编码器旋转计数  
  * @retval GPIO5 GPIO16
  */
const int8_t Encoder_IOA = 5;
const int8_t Encoder_IOB = 16;

int16_t Encoder_Count = 20;    // 编码器计数
// 编码器上一次值和当前值
int encoder_num_c = 0, encoder_num_p = 0;


/* 灯带部分程序设计 */
#define NUM_LEDS 4              // 灯珠数量
#define DATA_PIN 12             // GPIO12 控制信号引脚
uint8_t max_bright = 255*Encoder_Count/100;        // LED亮度控制变量，可使用数值为 0 ～ 255， 数值越大则光带亮度越高
uint8_t gHue = 0;               // 旋转许多图案使用的“基色”


CRGB leds[NUM_LEDS];    // 定义灯带

int MEDIA_PLAY_TIME = 0, MEDIA_PREVIOUS_TIME = 0;


/**
  * @brief  Arduino初始化
  * @param  Encoder_IOA：GPIO5   上拉输入
  * @param  Encoder_IOB：GPIO16  上拉输入
  * @retval 无
  */
void setup() {

    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // 灯带初始化
    FastLED.setBrightness(max_bright);                    // 设置光带亮度

    for(int i=0; i<NUM_LEDS; i++){
        leds[i] = leds[i] = CRGB(255, 0, 0);
        FastLED.show();
    }
    delay(500);
    for(int i=0; i<NUM_LEDS; i++){
        leds[i] = leds[i] = CRGB(0, 255, 0);
        FastLED.show();
    }
    delay(500);
    for(int i=0; i<NUM_LEDS; i++){
        leds[i] = leds[i] = CRGB(0, 0, 255);
        FastLED.show();
    }
    delay(500);
    FastLED.clear();
    FastLED.show();
    

    void alt(void);

    Serial.begin(115200);

    Serial.println("Starting BLE work!");

    // A B引脚  上拉输入
    pinMode(Encoder_IOA, INPUT_PULLUP);
    pinMode(Encoder_IOB, INPUT_PULLUP);

    
    attachInterrupt(digitalPinToInterrupt(Encoder_IOA), IOA_attachInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(Encoder_IOB), IOB_attachInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_INPUT), checkTicks, FALLING);

    button.attachClick(singleClick);
    button.attachDoubleClick(doubleClick);
    button.attachMultiClick(multiClick);

    button.setPressTicks(1000); 
    button.attachLongPressStart(pressStart);
    button.attachLongPressStop(pressStop);

    bleKeyboard.begin();
}



/**
  * @brief  Arduino主循环
  * @param  无
  * @retval 无
  */
void loop() 
{
    button.tick();

    if(bleKeyboard.isConnected()) 
    {
        // 一个来回扫过的彩色点，带有渐隐的轨迹
        fadeToBlackBy( leds, NUM_LEDS, 20);
        int pos = beatsin16( 13, 0, NUM_LEDS-1 );   // 13 为跑马的快慢
        leds[pos] += CHSV( gHue, 255, 192);
        EVERY_N_MILLISECONDS( 10 ) { gHue++; }
        FastLED.show();

        
        // 单击 播放/暂停
        if(single_state == 1) 
        {
            bleKeyboard.write(KEY_MEDIA_PLAY_PAUSE);
            for(int i=0; i<NUM_LEDS; i++){
                leds[i] = leds[i] = CRGB(0, 255, 0);
                FastLED.show();
            }
            MEDIA_PLAY_TIME = millis();
            delay(10);                           // 延时一下，防止数据没接收到
            bleKeyboard.releaseAll();            // 释放按键
            single_state = 0;
            delay(300);
            FastLED.clear();
            FastLED.show();
        }


        // 双击 播放上一曲
        if(double_state == 1) 
        {
            bleKeyboard.write(KEY_MEDIA_PREVIOUS_TRACK);
            for(int i=0; i<NUM_LEDS; i++){
                leds[i] = leds[i] = CRGB(0, 255, 0);
                FastLED.show();
            }
            MEDIA_PREVIOUS_TIME = millis();
            delay(10);                           // 延时一下，防止数据没接收到
            bleKeyboard.releaseAll();            // 释放按键
            double_state = 0; 

            delay(300);
            FastLED.clear();
            FastLED.show();
        }
        

        

        // 长按 循环切换上一曲
        if(PressState == 1) 
        {
            if(millis() - pressStartTime >= 1000) 
            {
                for(int i=0; i<NUM_LEDS; i++){
                    leds[i] = leds[i] = CRGB(0, 255, 0);
                    FastLED.show();
                }
                bleKeyboard.write(KEY_MEDIA_NEXT_TRACK);
                Serial.print("millis() - pressStartTime: ");
                Serial.println(millis() - pressStartTime);
                Serial.println("下一曲");                      // 下一曲
                
                delay(10);                                    // 延时一下，防止数据没接收到
                bleKeyboard.releaseAll();                     // 释放按键
                pressStartTime = millis();
                
                delay(300);
                FastLED.clear();
                FastLED.show();
            }
        }

        /* 
         *  编码器：
         *  右旋：音频音量增加
         *  左旋：音频音量减小
         */
        if(encoder_num_c != encoder_num_p)
        {
            if(encoder_num_c > encoder_num_p)
            {
                bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
                delay(10);
                Serial.println("vol++");
            }
            if(encoder_num_c < encoder_num_p)
            {
                bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN);
                delay(10);
                Serial.println("vol--");
            }
            encoder_num_p = encoder_num_c;
            bleKeyboard.releaseAll();
        }
    }

    // 蓝牙未连接双闪红灯
    if(bleKeyboard.isConnected() == 0)
    {
        for(int i=0; i<NUM_LEDS; i++){
            leds[i] = leds[i] = CRGB(255, 0, 0);
            FastLED.show();
        }
        delay(500);
        FastLED.clear();
        FastLED.show();
        delay(500);
    }
}



/**
  * @brief  IOA_attachInterrupt --- Encoder_IOA中断调用函数
  * @param  无
  * @retval 无
  */
void IOA_attachInterrupt() {
    if(digitalRead(Encoder_IOA) == 0) {
        if(digitalRead(Encoder_IOB) == 0) {
            Encoder_Count--;
            encoder_num_c = Encoder_Count;
            Serial.println(Encoder_Count);
        }
    }
}

/**
  * @brief  IOB_attachInterrupt --- Encoder_IOB中断调用函数
  * @param  无
  * @retval 无
  */
void IOB_attachInterrupt() {
    if(digitalRead(Encoder_IOB) == 0) {
        if(digitalRead(Encoder_IOA) == 0) {
            Encoder_Count++;
            encoder_num_c = Encoder_Count;
            Serial.println(Encoder_Count);
        }
    }
}
