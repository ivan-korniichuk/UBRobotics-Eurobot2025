#include <WiFi.h>
#include <esp_now.h>
#include <driver/adc.h>

#include "Hardware.h"
#include "robot.h"
#include "vl53l5cx_api.h"

/*

o o o o
  / \
   _

    
   
   .-> x
   |
   v y
   

   Turn- clockwise positive

   REENABLE LIDARS
*/

unsigned long lastLidar = 0, lastChange = 0, lastPacket;

VL53L5CX_Configuration* Dev;


uint8_t cmdBuffer[32];

struct moveCommand {
  int16_t velX;
  int16_t velY;
  int16_t turn;
};

moveCommand newCommand;

void onReceive(int len) {
  Serial.printf("onReceive[%d]: ", len);

  for (int i = 0; i < len; i++) {
    Serial.print(cmdBuffer[i]);
    Serial.print(" ");
  }

  Wire1.readBytes(cmdBuffer, len);

  if (cmdBuffer[1] == 1) {
    lastPacket = millis();
    // memcpy(&newCommand, cmdBuffer + 2, sizeof(moveCommand));

    newCommand.velX = (int16_t)cmdBuffer[2] << 8 | cmdBuffer[3];
    newCommand.velY = (int16_t)cmdBuffer[4] << 8 | cmdBuffer[5];
    newCommand.turn = (int16_t)cmdBuffer[6] << 8 | cmdBuffer[7];

    Serial.printf("velX=%d  velY=%d  turn=%d",
                  newCommand.velX,
                  newCommand.velY,
                  newCommand.turn);
  } else if (cmdBuffer[1] == 2) {
    robot::state = robot::SEEKING;
  }


  Serial.println();
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  esp_now_init();

  Wire.begin(40, 21, 400'000);

  hw::ensureI2CPins(40);
  Dev = new VL53L5CX_Configuration();

  if (vl53l5cx_init(Dev)) {
    Serial.println("L5 init failed");
    delay(1000);
    ESP.restart();
    while (true) { ESP.restart(); }
  }

  Serial.println("LIDAR on");


  delay(10);
  vl53l5cx_set_resolution(Dev, 64);
  delay(10);
  vl53l5cx_start_ranging(Dev);

  if (!hw::begin()) {
    Serial.println("HW init failed");
    delay(1000);
    ESP.restart();
    while (true) { ESP.restart(); }
  }






  Wire1.onReceive(onReceive);
  Wire1.begin(0x10, 1, 2, 400'000);

  robot::state = robot::SEEKING;
}

float rev = 20;

unsigned long delayTime;

uint8_t lastDir;
robot::LidarResult laze;

/*
  delayTime = millis();

  while (millis() - delayTime < 50) {
    robot::drive(0, 0, 0);
    hw::updateMotors();
    delay(1);
  }
*/
void loop() {

  while (true) {
    hw::updateMotors();
    robot::crawlToRelative(20, 0, 0, 250);
    while (!robot::crawlFinished) {
      hw::updateMotors();
      robot::crawlUpdate();
      Serial.println();
    }
    delay(100);
  }

  laze = robot::measureObject();
  if (laze.valid) {
    Serial.printf("Turning to %f \n", laze.angle);
    robot::crawlToRelative(0, 0, laze.angle, 250);
    while (!robot::crawlFinished) {
      hw::updateMotors();
      robot::crawlUpdate();
    }
  }




  laze = robot::measureObject();
  if (laze.valid) {

    Serial.printf("Moving to %f \n", laze.range);
    Serial.println(laze.range - 125);
    robot::crawlToRelative(0, -laze.range + 150, 0, 250);
    while (!robot::crawlFinished) {
      hw::updateMotors();
      robot::crawlUpdate();
    }
  }



  uint8_t isReadyCheck;
  uint8_t counts = 0;
  VL53L5CX_ResultsData dataGottem;
  uint8_t centered = 0;
  while (!centered) {

    /*
    laze = robot::measureObject();
    float angleError = laze.angle;
    if (laze.valid && fabs(angleError) > 5) {
      Serial.printf("Turning to %f \n", laze.angle);

      if (angleError > 0) {
        robot::crawlToRelative(0, 0, 2, 200);
      } else {
        robot::crawlToRelative(0, 0, -2, 200);
      }

      while (!robot::crawlFinished) {
        hw::updateMotors();
        robot::crawlUpdate();
      }
    }
    */



    /*
    laze = robot::measureObject();
    float rangeError = laze.range - 125;
    if (laze.valid && fabs(rangeError) > 40) {

      Serial.printf("Moving to %f \n", laze.range);

      if (rangeError > 0) {
        robot::crawlToRelative(0, -10, 0, 200);
      } else {
        robot::crawlToRelative(0, 10, 0, 200);
      }

      while (!robot::crawlFinished) {
        hw::updateMotors();
        robot::crawlUpdate();
      }
      delayTime = millis();

      while (millis() - delayTime < 1000) {
        robot::drive(0, 0, 0);
        hw::updateMotors();
        delay(1);
      }
    }
    */




    hw::ensureI2CPins(40);
    vl53l5cx_check_data_ready(Dev, &isReadyCheck);

    while (!isReadyCheck) {
      vl53l5cx_check_data_ready(Dev, &isReadyCheck);
    }

    if (!vl53l5cx_get_ranging_data(Dev, &dataGottem))  //Read distance data into array
    {
      counts = 0;
      for (int i = 32; i < 40; i++) {
        if (dataGottem.distance_mm[i] < 250 && dataGottem.nb_target_detected[i]) {
          counts++;
        }
      }

      Serial.println(counts);
    }


    if (counts != 4) {
      if (counts > 4) {
        //robot::crawlToRelative(-10, 0, 0, 1000);
        robot::drive(-50, 0, 0);
      } else {
        robot::drive(50, 0, 0);
        //robot::crawlToRelative(10, 0, 0, 1000);
      }
      while (!robot::crawlFinished) {
        hw::updateMotors();
        robot::crawlUpdate();
        //Serial.println("crawling");
      }
    } else {
      centered = 1;
    }
  }
  Serial.println("fully aligned");


  laze = robot::measureObject();

  robot::crawlToRelative(0, -laze.range + 80, 0, 1000);
  while (!robot::crawlFinished) {
    hw::updateMotors();
    robot::crawlUpdate();
  }

  while (true) {
    //robot::drive(0, 0, 0);
    hw::updateMotors();
  }

  hw::updateMotors();

  /* keep crawl running */
  robot::crawlUpdate();

  //uint8_t isReadyCheck;

  /*
  i2c_set_pin((i2c_port_t)0, (gpio_num_t)40, (gpio_num_t)21, GPIO_PULLUP_ENABLE, GPIO_PULLUP_ENABLE, I2C_MODE_MASTER);
  gpio_reset_pin((gpio_num_t)39);  // disconnect from matrix & clear config
  gpio_set_direction((gpio_num_t)39, GPIO_MODE_DISABLE);
  gpio_reset_pin((gpio_num_t)38);  // disconnect from matrix & clear config
  gpio_set_direction((gpio_num_t)38, GPIO_MODE_DISABLE);
  */

  /*
  hw::ensureI2CPins(40);
  vl53l5cx_check_data_ready(Dev, &isReadyCheck);
  if (isReadyCheck) {

    if (!vl53l5cx_get_ranging_data(Dev, &dataGottem))  //Read distance data into array
    {

      
      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0; y <= 5 * (8 - 2); y += 8) {
        for (int x = 8 - 1; x >= 0; x--) {
          Serial.print("\t");
          Serial.print(dataGottem.distance_mm[x + y]);
        }
        Serial.println();
      }
      Serial.println();
    }
  }
  */



  static float lSmooth = 0, rSmooth = 0, eSmooth = 0;

  if (millis() - lastLidar > 50) {

    // laze = robot::measureObject(10);
    //Serial.printf("Angle %f Range %f \n", laze.angle, laze.range);



    float batteryVoltage = 6 * 3.3 * ((float)adc1_get_raw(ADC1_CHANNEL_6)) / (4096.0);
    //Serial.printf("en1 %f en2 %f en3 %f %f \n", hw::motor1.relDeg(),hw::motor2.relDeg(),hw::motor3.relDeg(), batteryVoltage);

    //if (batteryVoltage < 6.8) {
    //  robot::drive(0, 0, 0);
    //}

    /*
    if (robot::overtimeWatchdog){
      robot::overtimeWatchdog = false;
      robot::crawlToRelative(0, 0, 0, 10)
    }
    */

    lastLidar = millis();
    if (robot::crawlFinished) {

      switch (robot::state) {
        case robot::DIRECT_CONTROL:
          /*
          if (millis() - lastPacket < 400) {
            robot::drive(4 * newCommand.velX, 4 * newCommand.velY, 4 * newCommand.turn);
          } else {
            robot::drive(0, 0, 0);
          }
          */
          break;

        case robot::SEEKING:
          Serial.println("SEEKING");
          laze = robot::measureObject();
          if (laze.valid) {
            robot::state = robot::ALIGNING;
          } else {
            robot::state = robot::ABORTING;
          }

          break;

        case robot::ALIGNING:
          Serial.println("ALIGNING");
          laze = robot::measureObject();
          Serial.printf("Correction %f \n", laze.angle);
          if (laze.valid) {
            if (abs(laze.angle) > 3) {
              Serial.println(laze.angle);
              robot::crawlToRelative(0, 0, laze.angle, 1000);
            } else {
              robot::state = robot::CLOSING;
              Serial.println("Aligned");
            }
          } else {
            robot::state = robot::ABORTING;
          }


          break;

        case robot::CLOSING:
          Serial.println("CLOSING");
          laze = robot::measureObject();

          if (laze.angle > 5) {
            //robot::crawlToRelative(0, 0, laze.angle, 150);

          } else {
            if (abs(laze.range - robot::cfg::pickupRange) > 4) {
              //robot::crawlToRelative(0, laze.range - robot::cfg::pickupRange, 0, 250);
            } else {
              //robot::state = robot::COARSE_STRAFE;
            }
          }

          break;


        case robot::COARSE_STRAFE:
          Serial.println("COARSE STRAFE");
          /*
          uint8_t isReadyCheck;
          vl53l5cx_check_data_ready(Dev, &isReadyCheck);
          if (isReadyCheck) {
            
            if (!vl53l5cx_get_ranging_data(Dev, &dataGottem))  //Read distance data into array
            {
              
              Serial.println(endTime - startTime);
              //The ST library returns the data transposed from zone mapping shown in datasheet
              //Pretty-print data with increasing y, decreasing x to reflect reality
              for (int y = 3; y <= imageWidth * (imageWidth - 2); y += imageWidth) {
                for (int x = imageWidth - 1; x >= 0; x--) {
                  Serial.print("\t");
                  Serial.print(dataGottem.distance_mm[x + y]);
                }
                Serial.println();
              }
              Serial.println();
              
            }
            */

          break;

        case robot::APPROACH:
          Serial.println("APPROACH");
          break;

        case robot::ABORTING:
          Serial.println("ABORTING");
          robot::state = robot::SEEKING;
          break;
      }
    }



    //Serial.printf("L:%u R:%u F:%u\n",l,r,f);
  }



  delay(1);
}
