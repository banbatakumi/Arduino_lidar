#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

#define SENSOR_NUM 16
#define RC 0.5

// #define LONG_RANGE
#define HIGH_SPEED

const int XSHUT_GPIO_ARRAY[SENSOR_NUM] = {2, 12, 11, 17, 16, 15, 14, 10, 13, 9, 8, 7, 6, 5, 4, 3};

VL53L0X tofSensor[SENSOR_NUM];

uint16_t value[SENSOR_NUM], rc_value[SENSOR_NUM];

void setup() {
      Serial.begin(38400);   // 通信速度: 9600, 14400, 19200, 28800, 38400, 57600, 115200
      Wire.begin();

      delay(500);   // 起動直後、I2Cが反応できるようになるまで待つ必要有り？！

      // 接続されているすべてのVL53L0Xを停止
      for (int i = 0; i < SENSOR_NUM; i++) {
            Serial.print("XSHUT_GPIO_ARRAY:");
            Serial.println(XSHUT_GPIO_ARRAY[i]);
            Serial.print("GPIO PORT ");
            Serial.print(XSHUT_GPIO_ARRAY[i]);
            Serial.println(" Set to OUTPUT and LOW");
            pinMode(XSHUT_GPIO_ARRAY[i], OUTPUT);
            delay(50);
            digitalWrite(XSHUT_GPIO_ARRAY[i], LOW);
            delay(50);
      }

      // 接続されているすべてのVL53L0Xを1つずつ起動して初期化
      for (int i = 0; i < SENSOR_NUM; i++) {
            Serial.print("Start Setup ");
            Serial.println(i);
            Serial.print("GPIO Port: ");
            Serial.println(XSHUT_GPIO_ARRAY[i]);
            digitalWrite(XSHUT_GPIO_ARRAY[i], HIGH);
            delay(100);   // default: 500

            Serial.print("tofSensor ");
            Serial.println(i);

            Serial.print(" Default Address is ");
            uint8_t retVal = tofSensor[i].getAddress();
            Serial.println(retVal);

            bool ret = false;
            while (ret == false) {
                  Serial.println(ret);
                  delay(100);
                  ret = tofSensor[i].init();
                  Serial.print("Try: ");
                  Serial.println(i);
                  if (ret == false) {
                        Serial.println("init false...retry");
                        digitalWrite(XSHUT_GPIO_ARRAY[i], HIGH);
                  } else {
                        Serial.println("init success!");
                  }
            }
            tofSensor[i].setTimeout(500);
            tofSensor[i].setAddress((uint8_t)20 + (i * 2));
            Serial.print(" changed Address to ");
            retVal = tofSensor[i].getAddress();
            Serial.println(retVal);

#if defined LONG_RANGE
            // lower the return signal rate limit (default is 0.25 MCPS)
            tofSensor[i].setSignalRateLimit(0.1);
            // increase laser pulse periods (defaults are 14 and 10 PCLKs)
            tofSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
            tofSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#elif defined HIGH_SPEED
            // reduce timing budget to 20 ms (default is about 33 ms)
            tofSensor[i].setMeasurementTimingBudget(20000);
#endif
      }
}

void loop() {
      for (uint8_t i = 0; i < SENSOR_NUM; i++) {
            value[i] = tofSensor[i].readRangeSingleMillimeters();
            if (value[i] > 800) value[i] = 800;
            value[i] *= 0.3;

            rc_value[i] = rc_value[i] * RC + value[i] * (1 - RC);
      }

      Serial.write(255);
      for (uint8_t i = 0; i < SENSOR_NUM; i++) {
            Serial.write(rc_value[i]);
      }
}