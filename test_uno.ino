#include "classes/motorControl.cpp"
#include "classes/PIDControl.cpp"
#include "classes/Gyro.cpp"

// Define PID parameters
const double Kp = 20; //Proportional path weight
const double Ki = 7; //Integration path weight
const double Kd = 0.3; //Derivation path weight

const int maxPower = 50; //max power of motor
const int cutoffAngle = 35; //angle after which the motor gets disbaled

boolean enabled = true;
double output = 1500;

//PID settings
const double minOut = 1000 + (500 - 500*maxPower/100), maxOut = 2000 - (500 - 500*maxPower/100);
int sampleTime = 10;
double setPoint = 0;
float input_Angle;

// Define gyro and PID control
PIDControl PIDRoll(Kp, Ki, Kd, minOut, maxOut, setPoint, sampleTime);
Gyro gyro;
motorControl controller;
/**
 * Method that is called in the beginning of excecution for setup
*/
void setup() {
  Serial.begin(115200);

  gyro.setup();
  controller.setupESC();
}


/**
 * Loop method that is used to run code in an infinite loop
*/
void loop() {
  gyro.loop();
  input_Angle = gyro.getRoll();

  output = PIDRoll.compute((double)input_Angle);

  if(abs(input_Angle) > cutoffAngle) enabled = false;

  Serial.print(input_Angle);
  Serial.print(" ");
  Serial.print(output);
  Serial.print(" ");
  Serial.println(enabled);

  if(enabled) controller.writeMicroseconds(output);
  else controller.writeMicroseconds(1500);
}

