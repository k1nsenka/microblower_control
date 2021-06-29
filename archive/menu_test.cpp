#include <vector>
#include <M5Stack.h>
#include <M5TreeView.h>
#include <Wire.h>
#include "utility/MPU9250.h"
#include "MAX30100.h"
#include "DHT12.h"
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>
#define SAMPLING_RATE   MAX30100_SAMPRATE_100HZ
#define IR_LED_CURRENT  MAX30100_LED_CURR_50MA
#define RED_LED_CURRENT MAX30100_LED_CURR_27_1MA
// set HIGHRES_MODE to true only
// when setting PULSE_WIDTH to MAX30100_SPC_PW_1600US_16BITS
#define PULSE_WIDTH MAX30100_SPC_PW_1600US_16BITS
#define HIGHRES_MODE    true

MPU9250 IMU;
MAX30100 sensor;
M5TreeView treeView;
DHT12 dht12;
Adafruit_BMP280 bme;

typedef std::vector<MenuItem*> vmi;

void (*update_func)() = NULL; //update_funcの中身の有無でスイッチさせる


//heartrate
void switch_to_heatrbeat(MenuItem* sender)
{
  update_func = H_loop;
   M5.Lcd.clear();
   Serial.print("Initializing MAX30100..");
   if (!sensor.begin()) {
      Serial.println("FAILED");
      for(;;);
   } else {
      Serial.println("SUCCESS");
   }
   sensor.setMode(MAX30100_MODE_SPO2_HR);
   sensor.setLedsCurrent(IR_LED_CURRENT, RED_LED_CURRENT);
   sensor.setLedsPulseWidth(PULSE_WIDTH);
   sensor.setSamplingRate(SAMPLING_RATE);
   sensor.setHighresModeEnabled(HIGHRES_MODE);
}

const int LCD_WIDTH = 320;
const int LCD_HEIGHT = 240;
const int DOTS_DIV = 30;
#define GREY 0x7BEF

void DrawGrid() {
    for (int x = 0; x <= LCD_WIDTH; x += 2) { // Horizontal Line
        for (int y = 0; y <= LCD_HEIGHT; y += DOTS_DIV) {
            M5.Lcd.drawPixel(x, y, GREY);
        }
        if (LCD_HEIGHT == 240) {
            M5.Lcd.drawPixel(x, LCD_HEIGHT - 1, GREY);
        }
    }
    for (int x = 0; x <= LCD_WIDTH; x += DOTS_DIV) { // Vertical Line
        for (int y = 0; y <= LCD_HEIGHT; y += 2) {
            M5.Lcd.drawPixel(x, y, GREY);
        }
    }
}

int lastMin = 70000, lastMax = 50000;
int minS= 70000, maxS = 50000;
int lastY = 50000;
int x = 0;
int count1 = 0;
int p = 0;
int lastp = 0;
int k[10];
int i = 0;
int bpm;
int flag = 0;
#define REDRAW 20 // msec

void H_loop(){
    M5.update();
    delay(REDRAW);
    uint16_t ir, red;
    sensor.update();
    while(sensor.getRawValues(&ir, &red)){
      Serial.println(ir);
      Serial.print('\t');
      Serial.println(red);
    };
    uint16_t y = red;
    if (y < minS) minS = y;
    if (maxS < y) maxS = y;
    ++i;
    p += y;
    if(i<=8){
    k[i-1]=y; //0~7の8要素に値を入れる
    if(i==8) lastp = p;
    } else {
      p -= k[(i-1)%8];
      k[(i-1)%8]=y;
      if(lastp>p+200){
        flag++;
        if(flag==1) ++count1;
      } else flag = 0;
      lastp = p;
      //Serial.print("   lastp: ");
      //Serial.println(lastp);
    }
        

    if (x > 0) {
        y = (int)(LCD_HEIGHT - (float)(y - lastMin) / (lastMax - lastMin) * LCD_HEIGHT);
        M5.Lcd.drawLine(x - 1, lastY, x, y, WHITE);
        lastY = y;
    }

    if (++x > LCD_WIDTH) {
        x = 0;
        M5.Lcd.fillScreen(BLACK);
        DrawGrid();
        if(minS>50000) lastMin = minS - 20;
        if(maxS<70000) lastMax = maxS + 20;
        minS = 70000;
        maxS = 50000;
        bpm = (float)count1/6.4 * 60;
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.setTextSize(4);
        M5.Lcd.printf("BPM: %d", bpm);
        count1 = 0;
        i = 0;
        Serial.print("lastmin: ");
        Serial.print(lastMin);
        Serial.print("   lastmax: ");
        Serial.println(lastMax);
    }
    if (M5.BtnA.isPressed()) {
      M5.Lcd.clear();
      update_func = NULL;
    }
}


//gyro
void switch_to_gyromode(MenuItem* sender)
{
  update_func = G_loop;
}

void G_loop() {
  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    M5.update();
    M5.Lcd.clear();
    M5.Lcd.setCursor(0, 0);
    IMU.readAccelData(IMU.accelCount);
    IMU.getAres(); // get accelerometer scales saved to "aRes"
    IMU.ax = (float)IMU.accelCount[0] * IMU.aRes; // - accelBias[0];
    IMU.ay = (float)IMU.accelCount[1] * IMU.aRes; // - accelBias[1];
    IMU.az = (float)IMU.accelCount[2] * IMU.aRes; // - accelBias[2];
    M5.Lcd.print("X-acceleration: "); M5.Lcd.print(1000 * IMU.ax);
    M5.Lcd.println(" mg ");
    M5.Lcd.print("Y-acceleration: "); M5.Lcd.print(1000 * IMU.ay);
    M5.Lcd.println(" mg ");
    M5.Lcd.print("Z-acceleration: "); M5.Lcd.print(1000 * IMU.az);
    M5.Lcd.println(" mg ");
    IMU.tempCount = IMU.readTempData();
    IMU.temperature = ((float) IMU.tempCount) / 333.87 + 21.0;
    M5.Lcd.print("MPU9250 Temperature is ");
    M5.Lcd.print(IMU.temperature, 1);

    Serial.print(IMU.ax);
    Serial.print(",");
    Serial.print(IMU.ay);
    Serial.print(",");
    Serial.println(IMU.az);
    if (M5.BtnA.isPressed()) {
      M5.Lcd.clear();
      update_func = NULL;
    }
    delay(200);
  }
}


//temperature
void switch_to_temperature(MenuItem* sender)
{
  update_func = T_loop;
}

void T_loop(){
    M5.update();
    M5.Lcd.clear();
    M5.Lcd.setCursor(0, 0);
   IMU.tempCount = IMU.readTempData();
   IMU.temperature = ((float) IMU.tempCount) / 333.87 + 21.0;
   M5.Lcd.setCursor(0, 0);
   M5.Lcd.setTextColor(WHITE);
   M5.Lcd.setTextSize(1);
   M5.Lcd.println("MPU9250 Temperature is ");
   M5.Lcd.setTextColor(YELLOW);
   M5.Lcd.setTextSize(6);
   M5.Lcd.setCursor(100, 100);
   M5.Lcd.print(IMU.temperature, 1);
   if (M5.BtnA.isPressed()) {
      M5.Lcd.clear();
      update_func = NULL;
   }
   delay(500);
  }


//environment
void switch_to_environment(MenuItem* sender)
{
  update_func = E_loop;
  M5.Lcd.clear();
  M5.Lcd.setBrightness(10);
  Serial.println(F("ENV Unit(DHT12 and BMP280) test..."));
  while (!bme.begin(0x76)){  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    M5.Lcd.println("Could not find a valid BMP280 sensor, check wiring!");
  }
  M5.Lcd.clear(BLACK);
  M5.Lcd.println("ENV Unit test...");
}

void E_loop(){
    M5.update();
    if (M5.BtnA.isPressed()) {
      update_func = NULL;
      M5.Lcd.clear();
    }
    float tmp = dht12.readTemperature();
    float hum = dht12.readHumidity();
    float pressure = bme.readPressure();
    Serial.printf("Temperatura: %2.2f*C  Humedad: %0.2f%%  Pressure: %0.2fPa\r\n", tmp, hum, pressure);

    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setTextSize(3);
    M5.Lcd.printf("Temp: %2.1f  \r\nHumi: %2.0f%%  \r\nPressure:%2.0fPa\r\n", tmp, hum, pressure);
    delay(200);
}



void setup() {
  M5.begin();
  Wire.begin();
  Serial.begin(115200);
  IMU.initMPU9250();
  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
  IMU.initAK8963(IMU.magCalibration);

  treeView.useFACES       = true;
  treeView.useCardKB      = true;
  treeView.useJoyStick    = true;
  treeView.usePLUSEncoder = true;
  treeView.useFACESEncoder= true;
  treeView.clientRect.x = 2;
  treeView.clientRect.y = 10;
  treeView.clientRect.w = 316;
  treeView.clientRect.h = 216;

  treeView.setItems(vmi
               { new MenuItem("heartbeat", switch_to_heatrbeat)
//                 { new MenuItem("sub 1-1", 11, vmi
//                   { new MenuItem("sub 1-1-1", 111)
//                   } )
//                 } )
               , new MenuItem("gyro", switch_to_gyromode)
//                 { new MenuItem("sub 2-1", vmi
//                   { new MenuItem("sub 2-1-1", 211)
//                   , new MenuItem("sub 2-1-2", 212)
//                   } )
//                 , new MenuItem("sub 2-2", vmi
//                   { new MenuItem("sub 2-2-1", 221)
//                   , new MenuItem("sub 2-2-2", 222)
//                   } )
//                 } )
               , new MenuItem("temperature", vmi
                   { new MenuItem("temperature", switch_to_temperature)
                   , new MenuItem("more environment", switch_to_environment)
//                     { new MenuItem("sub 3-1-2-1", 3121)
//                     , new MenuItem("sub 3-1-2-2", 3122)
//                     , new MenuItem("sub 3-1-2-3", 3123)
//                   } )
//                   , new MenuItem("sub 3-1-3", vmi
//                     { new MenuItem("sub 3-1-3-1", 3131)
//                     , new MenuItem("sub 3-1-3-2", 3132)
//                     , new MenuItem("sub 3-1-3-3", 3133)
//                     } )
//                   } )
//                 , new MenuItem("sub 3-2", vmi
//                   { new MenuItem("sub 3-2-1", vmi
//                     { new MenuItem("sub 3-2-1-1", 3211)
//                     , new MenuItem("sub 3-2-1-2", 3212)
//                     , new MenuItem("sub 3-2-1-3", 3213)
//                   } )
//                   , new MenuItem("sub 3-2-2", 322)
//                   , new MenuItem("sub 3-2-3", 323)
//                   } )
//                 , new MenuItem("sub 3-3", vmi
//                   { new MenuItem("sub 3-3-1", vmi
//                     { new MenuItem("sub 3-3-1-1", 3311)
//                     , new MenuItem("sub 3-3-1-2", 3312)
//                     , new MenuItem("sub 3-3-1-3", 3313)
//                   } )
//                   , new MenuItem("sub 3-3-2", 332)
//                   , new MenuItem("sub 3-3-3", vmi
//                     { new MenuItem("sub 3-3-3-1", 3331)
//                     , new MenuItem("sub 3-3-3-2", 3332)
//                     , new MenuItem("sub 3-3-3-3", 3333)
//                     } )
//                   } )
                 } )
               }
             );

  treeView.begin();
}

void loop() {
  if (update_func == NULL) {
    MenuItem* mi = treeView.update();
    if (mi != NULL) {
      M5.Lcd.fillRect(0,0,320,8,0);
      M5.Lcd.setTextColor(0xffff,0);
      M5.Lcd.setTextSize(1);
      M5.Lcd.drawString("menu:" + mi->title + " / tag:" + mi->tag, 15, 0, 1);
    }
    return;
  }
  update_func();
}