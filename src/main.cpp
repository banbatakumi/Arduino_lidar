
#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

#define SENSOR_QTY 16
#define RC 0.8
#define MAX_VAL 1100

const int XSHUT_GPIO_ARRAY[SENSOR_QTY] = {2, 12, 11, 17, 16, 15, 14, 10, 13, 9, 8, 7, 6, 5, 4, 3};

VL53L0X tofSensor[SENSOR_QTY];

uint16_t val[SENSOR_QTY];
uint16_t rc_val[SENSOR_QTY];
uint16_t offset[SENSOR_QTY] = {1, 2, 5, 3, 3, 2, 3, 5, 6, 3, 4, 6, 1, 0, 5, 14};

void setup() {
      Serial.begin(57600);   // 通信速度: 9600, 14400, 19200, 28800, 38400, 57600, 115200
      Wire.begin();
      Wire.setClock(400000);

      delay(100);   // 起動直後、I2Cが反応できるようになるまで待つ必要有り？！

      // 接続されているすべてのVL53L0Xを停止
      for (int i = 0; i < SENSOR_QTY; i++) {
            pinMode(XSHUT_GPIO_ARRAY[i], OUTPUT);
            delay(10);
            digitalWrite(XSHUT_GPIO_ARRAY[i], LOW);
            delay(10);
      }

      // 接続されているすべてのVL53L0Xを1つずつ起動して初期化
      for (int i = 0; i < SENSOR_QTY; i++) {
            digitalWrite(XSHUT_GPIO_ARRAY[i], HIGH);
            delay(10);

            bool ret = false;
            while (ret == false) {
                  delay(10);
                  ret = tofSensor[i].init();
                  if (ret == false) digitalWrite(XSHUT_GPIO_ARRAY[i], HIGH);
            }
            tofSensor[i].setTimeout(100);   // default: 500
            tofSensor[i].setAddress((uint8_t)20 + (i * 2));

            // 測定距離を長くする
            tofSensor[i].setSignalRateLimit(0.1);
            tofSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
            tofSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

            tofSensor[i].startContinuous();
      }
}

void loop() {
      for (uint8_t i = 0; i < SENSOR_QTY; i++) {
            val[i] = tofSensor[i].readRangeContinuousMillimeters();
            if (val[i] > MAX_VAL) val[i] = MAX_VAL;
            val[i] *= 200.00 / MAX_VAL;
            val[i] -= offset[i];

            rc_val[i] = rc_val[i] * RC + val[i] * (1 - RC);
      }

      Serial.write(0xFF);
      for (uint8_t i = 0; i < SENSOR_QTY; i++) {
            Serial.write(rc_val[i]);
      }
      Serial.write(0xAA);
      Serial.flush();
      
      /*
      for (uint8_t i = 0; i < SENSOR_QTY; i++) {
            Serial.print(" ");
            Serial.print(i);
            Serial.print(":");
            Serial.print(rc_val[i]);
      }
      Serial.println();*/
}