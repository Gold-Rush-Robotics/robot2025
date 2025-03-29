
#include <string>
#include <stdio.h>
#include <vector>

#include "GRRServo.h"
#include "GRRMotor.h"
#include "GRRCan.h"

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

// create motor vector
std::vector<GRRMotor> motors;
// create servo vector
std::vector<GRRServo> servos;

uint8_t serial[4];

String serialnumber = teensySN();
// remove the - in serial number
enum CAN_IDs
{
  E_STOP = 0x000,
  HALT = 0x100,
  RESTART = 0x200,
  HEARTBEAT = 0x300,
  QUERY = 0x400,
  ASSIGN_ID = 0x500,
  FIRMWARE = 0x600,
  FATAL = 0x1000,
  ERROR = 0x2000,
  MOTOR_COMMAND = 0x3000,
  SERVO_CONTROL = 0x4000,
  DIO = 0x5000,
  SENSORS = 0x6000,
  WARNINGS = 0x7000,
  LOGS = 0x8000,
  MOANING = 0xFFFE,
  SGA_WARRANTY = 0xFFFF
};

GRRMotor::GRRMotor(int DIR, int PWM, int SLP, int FLT, int EN_OUTA, int EN_OUTB, int CS, String joint_name, String control_type)
{
  this->DIR = DIR;
  this->PWM = PWM;
  this->SLP = SLP;
  this->FLT = FLT;
  this->EN_OUTA = EN_OUTA;
  this->EN_OUTB = EN_OUTB;
  this->CS = CS;
  this->joint_name = joint_name;
  this->control_type = control_type;
  pinMode(DIR, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(SLP, OUTPUT);
  pinMode(FLT, INPUT);
  pinMode(EN_OUTA, INPUT);
  pinMode(EN_OUTB, INPUT);
  pinMode(CS, INPUT);
}
GRRMotor::GRRMotor()
{
  this->joint_name = "NULL";
  this->control_type = "NULL";
}
void GRRMotor::setMotorVelocity(float velocity)
{
  digitalWrite(SLP, HIGH); // Wake up the motor driver
  if (velocity >= 0)
  {
    digitalWrite(DIR, HIGH); // Set direction forward
  }
  else
  {
    digitalWrite(DIR, LOW); // Set direction reverse
    velocity = -velocity;   // Make speed positive
  }
  analogWrite(PWM, velocity); // Set motor speed
}
void GRRMotor::setMotorEffort(int effort)
{
  digitalWrite(SLP, HIGH); // Wake up the motor driver
  if (effort >= 0)
  {
    digitalWrite(DIR, HIGH); // Set direction forward
  }
  else
  {
    digitalWrite(DIR, LOW); // Set direction reverse
    effort = -effort;       // Make speed positive
  }
  analogWrite(PWM, effort); // Set motor speed
}
float GRRMotor::getMotorVelocity()
{
  if (abs(analogRead(EN_OUTA)) > abs(analogRead(EN_OUTB)))
  {
    return analogRead(EN_OUTA);
  }
  else
  {
    return analogRead(EN_OUTB);
  }
}
int GRRMotor::getMotorEffort()
{
  if (abs(analogRead(EN_OUTA)) > abs(analogRead(EN_OUTB)))
  {
    return analogRead(EN_OUTA);
  }
  else
  {
    return analogRead(EN_OUTB);
  }
}
String GRRMotor::getJointName()
{
  return joint_name;
}
String GRRMotor::getControlType()
{
  return control_type;
}

GRRServo::GRRServo(int port, int upper_limit, int lower_limit, String joint_name)
{
  this->port = port;
  this->upper_limit = upper_limit;
  this->lower_limit = lower_limit;
  this->joint_name = joint_name;
  pinMode(port, OUTPUT);
  servo.attach(port);
}
void GRRServo::setServoPosition(int position)
{
  if (upper_limit >= position && lower_limit <= position)
  {
    servo.write(position);
  }
}
String GRRServo::getJointName()
{
  return joint_name;
}

FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16>
    flexCan;

// void printInfo(const CANFD_message_t &message, bool received = true, bool result = true)
// {
//   Serial.println("*******************************");
//   Serial.println(received ? "* Frame Received" : "* Frame Send");
//   Serial.println(result ? "* Success" : "* Fail");
//   Serial.println("*");
//   Serial.print("* Id: ");
//   Serial.print(message.id, HEX);
//   Serial.println();
//   Serial.print("* Data: ");
//   for (int i = 0; i < message.len; i++)
//   {
//     Serial.print(message.buf[i], HEX);
//     Serial.print(" ");
//   }
//   Serial.println();
//   Serial.println("*");
//   Serial.println("*******************************");
// }

// void frameReceivedHandler(const CANFD_message_t &message)
// {
//   printInfo(message);
// }

int txId = 0x124, rxId = 0x124;

// int txId = 0x321, rxId = 0x123;

bool last_state = true;

void setup(void)
{
  Serial.begin(9600);
  String cleanedSerialNumber = "";
  for (size_t i = 0; i < serialnumber.length(); i++)
  {
    if (serialnumber[i] != '-')
    {
      cleanedSerialNumber += serialnumber[i];
    }
  }
  serialnumber = cleanedSerialNumber;

  /************ Initialize CAN ************/
  CANFD_timings_t config;
  config.clock = CLK_20MHz;
  config.baudrate = 1000 * 1000;
  config.baudrateFD = 2000000;
  config.propdelay = 190;
  config.bus_length = 1;
  config.sample = 70;
  flexCan.begin();
  flexCan.setRegions(128);
  flexCan.setBaudRate(config);

  // flexCan.setBaudRate(500000);
}
bool sendMessage(CANFD_message_t &message)
{
  static unsigned long timeout = millis();
  int result;
  if (millis() - timeout >= 500)
  {
    timeout = millis();

    result = flexCan.write(message);
    // printInfo(message, false, result);
    // flexCan.mailboxStatus();
    return false;
  }
  return true;
}
void sendHeartBeat()
{
  CANFD_message_t message;
  message.id = HEARTBEAT;
  message.len = 16;
  message.buf[0] = (int)serialnumber[0];
  message.buf[1] = (int)serialnumber[1];
  message.buf[2] = (int)serialnumber[2];
  message.buf[3] = (int)serialnumber[3];
  message.buf[4] = (int)serialnumber[4];
  message.buf[5] = (int)serialnumber[5];
  message.buf[6] = (int)serialnumber[6];
  message.buf[7] = (int)serialnumber[7];
  message.buf[8] = 0;
  message.buf[9] = 0;
  message.buf[10] = 0;
  message.buf[11] = 0;
  message.buf[12] = 0;
  message.buf[13] = 0;
  message.buf[14] = 0;
  message.buf[15] = 0;
  sendMessage(message);
}
bool checkIfSerialNumberMatches(CANFD_message_t &message, String serialnumber)
{
  String recievedserialnumber = "";
  for (int i = 0; i < 8; i++)
  {
    recievedserialnumber = recievedserialnumber + char(message.buf[i]);
  }
  if (serialnumber != recievedserialnumber)
  {
    Serial.println("Serial number does not match");
    Serial.println(serialnumber);
    Serial.println(recievedserialnumber);
    return false;
  }
  else
  {
    return true;
  }
}
String getJointName(CANFD_message_t &message)
{
  String joint_name;
  for (int i = 15; i < 31; i++)
  {
    joint_name = joint_name + char(message.buf[i]);
  }
  return joint_name;
}
String getControlType(CANFD_message_t &message)
{
  String control_type;
  for (int i = 31; i < 39; i++)
  {
    control_type = control_type + char(message.buf[i]);
  }
  Serial.print("Extracted control_type: ");
  Serial.println(control_type); // Debug print
  return control_type;
}
float getControlValue(CANFD_message_t &message)
{
  float control_value = 0.0;
  for (int i = 9; i < 15; i++)
  {
    control_value = control_value + char(message.buf[i]);
  }
  return float(control_value);
}
void setControlElements(CANFD_message_t &message, String serialnumber)
{
  if (checkIfSerialNumberMatches(message, serialnumber))
  {
    int buf1 = int(message.buf[9]);
    int buf2 = int(message.buf[10]);
    int buf3 = int(message.buf[11]);
    int buf4 = int(message.buf[12]);
    int buf5 = int(message.buf[13]);
    int buf6 = int(message.buf[14]);
    int buf7 = int(message.buf[15]);
    String joint_name = getJointName(message);
    String control_type = getControlType(message);

    if (control_type.equals("velocity"))
    {
      GRRMotor motor = GRRMotor(buf1, buf2, buf3, buf4, buf5, buf6, buf7, joint_name, control_type);
      motors.push_back(motor);
    }
    else if (control_type.equals("position"))
    {
      GRRServo servo = GRRServo(buf5, buf6, buf7, joint_name);
      servos.push_back(servo);
    }
    else if (control_type.equals("effort"))
    {
      GRRMotor motor = GRRMotor(buf1, buf2, buf3, buf4, buf5, buf6, buf7, joint_name, control_type);
      motors.push_back(motor);
    }
    else
    {
      Serial.println("Invalid control type");
    }
  }
}

void loop()
{
  sendHeartBeat();
  flexCan.events();
  CANFD_message_t message;
  /************ Call event handlers ************/
  if (flexCan.read(message))
  {
    // frameReceivedHandler(message);
    // check if message id is in CAN_IDs
    // if it is, call the appropriate function
    if (message.id == E_STOP)
    {
      // call e-stop function
    }
    else if (message.id == HALT)
    {
      // call halt function
    }
    else if (message.id == RESTART)
    {
      // call restart function
    }
    else if (message.id == HEARTBEAT)
    {
      // call heartbeat function
    }
    else if (message.id == QUERY)
    {
      // call query function
    }
    else if (message.id == ASSIGN_ID)
    {
      setControlElements(message, serialnumber);
    }
    else if (message.id == FIRMWARE)
    {
      // call firmware function
    }
    else if (message.id == MOTOR_COMMAND)
    {
      if (checkIfSerialNumberMatches(message, serialnumber))
      {
        for (uint i = 0; i < motors.size(); i++)
        {
          if (motors[i].getJointName() == getJointName(message))
          {
            if (motors[i].getControlType() == "velocity")
            {
              motors[i].setMotorVelocity(getControlValue(message));
            }
            else if (motors[i].getControlType() == "effort")
            {
              motors[i].setMotorEffort(getControlValue(message));
            }

            else
            {
              Serial.println("Invalid control type");
            }
          }
        }
      }
    }
    else if (message.id == SERVO_CONTROL)
    {
      if (checkIfSerialNumberMatches(message, serialnumber))
      {
        for (uint i = 0; i < servos.size(); i++)
        {
          if (servos[i].getJointName() == getJointName(message))
          {
            servos[i].setServoPosition(getControlValue(message));
          }
        }
      }
    }
    else if (message.id == DIO)
    {
      // call DIO function
    }
    else if (message.id == SENSORS)
    {
      // call sensors function
    }
    else if (message.id == WARNINGS)
    {
      // call warnings function
    }
    else if (message.id == LOGS)
    {
      // call logs function
    }
    else if (message.id == FATAL)
    {
      // call fatal function
    }
    else if (message.id == ERROR)
    {
      // call error function
    }
    else if (message.id == MOANING)
    {
      // call moaning function
    }
    else if (message.id == SGA_WARRANTY)
    {
      // call SGA warranty function
    }
  }
}
