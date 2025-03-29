#include <Servo.h>

Servo myServo1;


int pos = 0;

void setup() {
  // put your setup code here, to run once:
  myServo1.attach(3);
  
  myServo1.write(0);

}

void loop() {
  // put your main code here, to run repeatedly:
  for (pos = 0; pos < 156; pos+= 1) {
    myServo1.write(pos);
    delay(50);
  } 
  for (pos = 156; pos > 0; pos -= 1) {
    myServo1.write(pos);
    delay(50);
  }

}
