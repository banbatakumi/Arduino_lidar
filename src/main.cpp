
#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

#define SENSOR_QTY 16
#define RC 0.5
#define MAX_VAL 1000
#define OUT_RANGE_VAL 8000
#define OUT_RANGE_RESET_NUM 100
#define OUT_RAMGE_SENSOR_QTY 8

// #define LONG_RANGE
// #define HIGH_SPEED
// #define HIGH_ACCURACY

VL53L0X tof[SENSOR_QTY];

const int XSHUT_GPIO_ARRAY[SENSOR_QTY] = {2, 12, 11, 17, 16, 15, 14, 10, 13, 9, 8, 7, 6, 5, 4, 3};
const char FIRST_ADDRESS = 0x30;

uint16_t val[SENSOR_QTY];
uint8_t rc_val[SENSOR_QTY];
uint8_t out_range_qty;
uint8_t out_range_cnt;

bool which_data_send;

void (*resetFunc)(void) = 0;

void tofInit() {
      // 接続されているすべてのVL53L0Xを停止
      for (int i = 0; i < SENSOR_QTY; i++) {
            pinMode(XSHUT_GPIO_ARRAY[i], OUTPUT);
      }

      // 接続されているすべてのVL53L0Xを1つずつ起動して初期化
      for (int i = 0; i < SENSOR_QTY; i++) {
            pinMode(XSHUT_GPIO_ARRAY[i], INPUT);
            delay(5);

            bool ret = false;
            while (ret == false) {
                  delay(10);
                  ret = tof[i].init();
                  if (ret == false) pinMode(XSHUT_GPIO_ARRAY[i], INPUT);
            }
            tof[i].setTimeout(100);   // default: 500
            tof[i].setAddress(FIRST_ADDRESS + i);   // それぞれのセンサにアドレスと設定

#if defined LONG_RANGE
            // lower the return signal rate limit (default is 0.25 MCPS)
            tof[i].setSignalRateLimit(0.1);
            // increase laser pulse periods (defaults are 14 and 10 PCLKs)
            tof[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
            tof[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
            // reduce timing budget to 20 ms (default is about 33 ms)
            tof[i].setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
            // increase timing budget to 200 ms
            tof[i].setMeasurementTimingBudget(200000);
#endif

            tof[i].startContinuous();
      }
}

void setup() {
      Serial.begin(115200);   // 通信速度: 9600, 14400, 19200, 28800, 38400, 57600, 115200
      Wire.begin();
      Wire.setClock(50000);

      // 内部のプルアップ抵抗をオフ
      pinMode(SDA, INPUT);
      pinMode(SCL, INPUT);

      tofInit();

      TIMSK0 = 0;   // Timer0の停止
}

void loop() {
      while (1) {   // 呼び出しのオーバーヘッド節減
            out_range_qty = 0;
            for (uint8_t i = 0; i < SENSOR_QTY; i++) {
                  val[i] = tof[i].readRangeContinuousMillimeters();
                  if (val[i] > OUT_RANGE_VAL) out_range_qty++;   // レンジオーバーのセンサをカウント
                  if (val[i] > MAX_VAL) val[i] = MAX_VAL;   // レンジオーバーの場合最大値に戻す
                  val[i] *= 0.2;   // cmに変換
                  rc_val[i] = rc_val[i] * RC + val[i] * (1 - RC);
            }

            if (out_range_qty > OUT_RAMGE_SENSOR_QTY) {
                  out_range_cnt++;
            } else {
                  out_range_cnt = 0;
            }

            if (out_range_cnt > OUT_RANGE_RESET_NUM) tofInit();   // エラーを起こしたセンサが一定数を超えたらソフトウェアリセット

            // UART送信
            which_data_send = 1 - which_data_send;
            if (which_data_send == 0) {
                  Serial.write(0xAF);
                  for (uint8_t i = 0; i < 8; i++) {
                        Serial.write(rc_val[i * 2]);
                  }
            } else {
                  Serial.write(0xFF);
                  for (uint8_t i = 0; i < 8; i++) {
                        Serial.write(rc_val[i * 2 + 1]);
                  }
            }
            Serial.write(0xAA);
            Serial.flush();

            /*
            for (uint8_t i = 0; i < SENSOR_QTY; i++) {
                  Serial.print(" ");
                  Serial.print(i);
                  Serial.print(":");
                  Serial.print(tof[i].readRangeContinuousMillimeters());
            }
            Serial.println();
            */
      }
}