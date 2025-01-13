#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();

void calibrate_servos(char low[], char high[]);

char low[9];
char high[9];

void setup() {
  driver.begin();
  driver.setPWMFreq(100);
  Serial.begin(9600);
}

void loop() {

}

void calibrate_servos(char low[], char high[]) {
  for (int i = 0; i < 9; i++) {
    driver.writeMicroseconds(i, 670);
    
    Serial.print("Does Servo ");
    Serial.print(i + 1);
    Serial.println(" reach full range? y/n... ");
    
    delay(1000); 
  }
}
