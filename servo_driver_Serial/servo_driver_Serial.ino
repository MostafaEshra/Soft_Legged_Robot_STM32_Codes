#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define servo1 0
#define servo2 1
#define servo3 2
#define servo4 3
#define servo5 4
#define servo6 5
#define servo7 6
#define servo8 7
#define servo9 8

const int MAX_ARRAY_SIZE = 12; // Maximum number of elements in the array
int myArray[MAX_ARRAY_SIZE];   // Array to store the input values
int arraySize = 0;             // Number of elements in the array

void parseInput(String input);

Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();

void Servo_2_Angle(char servo,int angle);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin(PB3, PB10); // PB3 SDA, PB10 SCL

  driver.begin();
  driver.setPWMFreq(50);

  Wire.setClock(400000);
}

//90, 670 @ 100 PWMFreq not sure !!!!
// 670, 670, 670, 670, 670, 670, 670, 670, 670 
// 90, 90, 90, 90, 90, 90, 90, 90, 90

// @ 50 PWMFreq
// 370, 2770
// 375, 370, 370, 375, 375, 375, 375, 375, 375
// 2770, 2770, 2770, 2770, 2770, 2770, 2770, 2770, 2770

// Servo 1: 370 : 2770
// Servo 2: 370 : 2770 ***
// Servo 3: 370 : 2770
// Servo 4: 375 : 2780 ***
// Servo 5: 375 : 2785
// Servo 6: 375 : 2770
// Servo 7: 375 : 2770
// Servo 8: 375 : 2785 ???
// Servo 9: 375 : 2780 

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    Serial.println("HI");
    // Read the input string from the serial buffer
    String input = Serial.readStringUntil('\n');

    // Parse the input string into the array
    parseInput(input);
    
    for (int i = 0; i < arraySize; i++) {
      // driver.writeMicroseconds(i, myArray[i]);
      Servo_2_Angle(i, myArray[i]);
    }
  }
}

void parseInput(String input) {
  arraySize = 0; // Reset the array size
  int startIndex = 0;
  int endIndex = input.indexOf(','); // Find the first comma

  // Extract numbers separated by commas
  while (endIndex > 0 && arraySize < MAX_ARRAY_SIZE) {
    myArray[arraySize++] = input.substring(startIndex, endIndex).toInt();
    startIndex = endIndex + 1;
    endIndex = input.indexOf(',', startIndex);
  }

  // Add the last number (or only number if no commas were found)
  if (arraySize < MAX_ARRAY_SIZE) {
    myArray[arraySize++] = input.substring(startIndex).toInt();
  }
}


void Servo_2_Angle(char servo,int angle) {
  int val;
  switch(servo) {
    case servo1:
      val = map(angle, 0, 180, 375, 2770);
      break;

    case servo2:
      val = map(angle, 0, 180, 375, 2770);
      break;

    case servo3:
      val = map(angle, 0, 180, 375, 2770);
      break;

    case servo4:
      val = map(angle, 0, 180, 375, 2775); // 375, 2780
      break;

    case servo5:
      val = map(angle, 0, 180, 375, 2785);
      break;

    case servo6:
      val = map(angle, 0, 180, 375, 2770);
      break;

    case servo7:
      val = map(angle, 0, 180, 375, 2770);
      break;

    case servo8:
      val = map(angle, 0, 180, 375, 2785);
      break;

    case servo9:
      val = map(angle, 0, 180, 375, 2780);
      break;
  }
  driver.writeMicroseconds(servo, val);
}