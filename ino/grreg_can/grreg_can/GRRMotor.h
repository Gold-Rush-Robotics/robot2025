class GRRMotor {
  private:
    int DIR = 0;
    int PWM = 0;
    int SLP = 0;
    int FLT = 0;
    int EN_OUTA = 0;
    int EN_OUTB = 0;
    int CS = 0;
    String joint_name = "";
    String control_type = "";
  public:
    GRRMotor(int DIR, int PWM, int SLP, int FLT, int EN_OUTA, int EN_OUTB, int CS, String joint_name, String control_type);
    GRRMotor();
    void setMotorVelocity(float velocity);
    void setMotorEffort(int effort);
    float getMotorVelocity();
    int getMotorEffort();
    String getJointName();
    String getControlType();
};