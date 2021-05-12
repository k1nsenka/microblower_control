#include<M5StackUpdater.h>
#include<M5Stack.h>
void setup() {
  M5.begin();
  if(digitalRead(BUTTON_A_PIN) == 0) {
    Serial.println("Will Load menu binary");
    updateFromFS(SD);
    ESP.restart();
  }
  M5.Lcd.drawString("Hello, world", 140, 160);
}

void loop() {


}
