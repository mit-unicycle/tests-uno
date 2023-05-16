#include <PID_v1.h>
#include <Arduino.h>

/**
 * Wrapper class for the PID library
 * 
 * Libraries used: 
 *  - PID by Brett Beauregard
 * 
 * Written by David Farah
*/
class PIDControl {
  private:
    double kp;    // Proportional gain
    double ki;    // Integral gain
    double kd;    // Derivative gain
    double minOut;    // Minimum output value
    double maxOut;    // Maximum output value
    double setpoint;  // Setpoint value
    double input;     // Input value
    double output;    // Output value

    PID pid;    // PID object

  public:
    PIDControl(double kp, double ki, double kd, double minOut, double maxOut, double setPoint, int sampleTime) : 
      kp(kp), ki(ki), kd(kd), minOut(minOut), maxOut(maxOut), setpoint(setPoint), pid(&input, &output, &setpoint, kp, ki, kd, DIRECT) {
        pid.SetOutputLimits(minOut, maxOut);
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(sampleTime);
      }

    void setSetpoint(double setpoint) {
      this->setpoint = setpoint;
    }

    void setTunings(double Kp, double Ki, double Kd){
      pid.SetTunings(Kp, Ki, Kd);
    }

    void setSampleTime(int NewSampleTime){
      pid.SetSampleTime(NewSampleTime);
    }

    double compute(double input) {
      this->input = input;
      pid.Compute();
      return this->output;
    }
};