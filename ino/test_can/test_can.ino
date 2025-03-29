// Pin definitions for Motor 1

#include <Servo.h> 
#define DIR1 1
#define PWM1 2
#define SLP1 7
#define FLT1 8
#define EN_OUTA1 11
#define EN_OUTB1 12
#define CS1 23

// Pin definitions for Motor 2
#define DIR2 29
#define PWM2 28
#define SLP2 34
#define FLT2 35
#define EN_OUTA2 24
#define EN_OUTB2 25
#define CS2 40

 
Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
Servo myservo6;
 
int pos = 0;    // variable to store the servo position 
 

// Target RPM for motors
const int targetRPM = 0;

// Function to initialize motor pins
void setupMotorPins() {
  // Motor 1 pins
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(SLP1, OUTPUT);
  pinMode(FLT1, INPUT);
  pinMode(EN_OUTA1, INPUT);
  pinMode(EN_OUTB1, INPUT);
  pinMode(CS1, INPUT);

  // Motor 2 pins
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(SLP2, OUTPUT);
  pinMode(FLT2, INPUT);
  pinMode(EN_OUTA2, INPUT);
  pinMode(EN_OUTB2, INPUT);
  pinMode(CS2, INPUT);

 /* Motor 3 pins 
  pinMode(DIR2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(SLP2, OUTPUT);
  pinMode(FLT2, INPUT);
  pinMode(EN_OUTA2, INPUT);
  pinMode(EN_OUTB2, INPUT);
  pinMode(CS2, INPUT);*/

  myservo1.attach(3);
  myservo2.attach(4);
  myservo3.attach(5);
  myservo4.attach(6);
  myservo5.attach(9);
  myservo6.attach(33);
  analogWriteFrequency(3,50);
  analogWriteFrequency(4,50);
  analogWriteFrequency(5,50);
  analogWriteFrequency(6,50);
  analogWriteFrequency(9,50);
  analogWriteFrequency(33,50);
}

// Function to set motor speed and direction
void setMotorSpeed(int dirPin, int pwmPin, int slpPin, int speed) {
  digitalWrite(slpPin, HIGH); // Wake up the motor driver
  if (speed >= 0) {
    digitalWrite(dirPin, HIGH); // Set direction forward
  } else {
    digitalWrite(dirPin, LOW); // Set direction reverse
    speed = -speed; // Make speed positive
  }
  analogWrite(pwmPin, speed); // Set motor speed
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Initialize motor pins
  setupMotorPins();

  // Wake up both motors
  digitalWrite(SLP1, HIGH);
  digitalWrite(SLP2, HIGH);

 delay(5000);

  // Set both motors to 100 RPM
  // Assuming a mapping of RPM to PWM value (e.g., 100 RPM = 128 PWM)
  setMotorSpeed(DIR1, PWM1, SLP1, map(100, -100, 100, -255, 255));
  setMotorSpeed(DIR2, PWM2, SLP2, map(-100, -100, 100, -255, 255));
  //setMotoSpeed(37, 36, map(100, );
}
//2 left bin grabber 87
void loop() {
  // myservo6.write(0);
  // delay(1000);
  // myservo1.write(-8);
  // delay(1000);
  // delay(1000);
  // myservo3.write(8);
  // delay(1000);
  // myservo4.write(8);
  // delay(1000);
  // myservo5.write(8);
  // delay(1000);
  // myservo6.write(8);
  // delay(1000);
  // myservo1.write(-8);
  // delay(1000);
  // myservo2.write(-8);
  // delay(1000);
  // myservo3.write(-8);
  // delay(1000);
  // myservo4.write(-8);
  // delay(1000);
  // myservo5.write(-8);
  // delay(1000);
  // myservo6.write(-8);
  // delay(1000);

  for(pos = -50; pos < 50; pos += 1)  // goes from 10 degrees to 170 degrees 
  {                                  // in steps of 1 degree 
    myservo2.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  delay(100);
  for(pos =50; pos>=-50; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo2.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  delay(100);
  // // Monitor fault pins for debugging
  // if (digitalRead(FLT1) == LOW) {

  //   Serial.println("Fault detected on Motor 1!");
  // }
  // if (digitalRead(FLT2) == LOW) {
  //   Serial.println("Fault detected on Motor 2!");
  // }

  // Add additional logic if needed
}
