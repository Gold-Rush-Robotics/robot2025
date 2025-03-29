#include <Servo.h>
#include <string>
class GRRServo {
  private:
    int port = 0;
    int upper_limit = 0;
    int lower_limit = 0;
    String joint_name = "";
    Servo servo;
  public:
    GRRServo(int port, int upper_limit, int lower_limit, String joint_name);
    void setServoPosition(int position);
    String getJointName();

};