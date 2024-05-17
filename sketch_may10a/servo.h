#ifndef servo_h
#define servo_h



class servo {
  private:
    int angle;
    int channel;

  public:
    servo(int iniAngle, int iniChannel);
    int mapDutyCycleAzul();
    int mapDutyCycleRojo();
    int getAngle();
    void setAngle(int newAngle, char color);

};

#endif