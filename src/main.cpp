#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

#define SENSOR_NUM 16
#define RC 0

// #define LONG_RANGE
#define HIGH_SPEED

const int XSHUT_GPIO_ARRAY[SENSOR_NUM] = {2, 12, 11, 17, 16, 15, 14, 10, 13, 9, 8, 7, 6, 5, 4, 3};

VL53L0X tofSensor[SENSOR_NUM];

uint16_t value[SENSOR_NUM], rc_value[SENSOR_NUM];

void setup() {
      Serial.begin(57600);   // 通信速度: 9600, 14400, 19200, 28800, 38400, 57600, 115200
      Wire.begin();
      Wire.setClock(400000);

      delay(500);   // 起動直後、I2Cが反応できるようになるまで待つ必要有り？！

      // 接続されているすべてのVL53L0Xを停止
      for (int i = 0; i < SENSOR_NUM; i++) {
            pinMode(XSHUT_GPIO_ARRAY[i], OUTPUT);
            delay(10);
            digitalWrite(XSHUT_GPIO_ARRAY[i], LOW);
            delay(10);
      }

      // 接続されているすべてのVL53L0Xを1つずつ起動して初期化
      for (int i = 0; i < SENSOR_NUM; i++) {
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
            tofSensor[i].setMeasurementTimingBudget(20000);
      }
}

void loop() {
      for (uint8_t i = 0; i < SENSOR_NUM; i++) {
            value[i] = tofSensor[i].readRangeSingleMillimeters();
            if (value[i] > 800) value[i] = 800;
            value[i] *= 0.25;

            rc_value[i] = rc_value[i] * RC + value[i] * (1 - RC);
      }

      Serial.write('H');
      for (uint8_t i = 0; i < SENSOR_NUM; i++) {
            Serial.write(rc_value[i]);
      }
}