#include <PID_AutoTune_v0.h>

// Create an instance of the PID_AT class
PID_AT pid;

void setup() {
  // Initialize the PID controller with the desired input and output ranges
  pid.SetInputLimits(0, 100);
  pid.SetOutputLimits(0, 255);

  // Set the sample time and controller direction
  pid.SetSampleTime(100);
  pid.SetControllerDirection(DIRECT);

  // Set the initial PID values (these can be any value)
  pid.SetTunings(1, 0, 0);
}

void loop() {
  // Call the Runtime() function to initiate the tuning process
  pid.Runtime();

  // Check the status of the tuning process
  if (pid.GetStatus() == TUNING_COMPLETE) {
    // The tuning process is complete, so get the optimized PID values
    double Kp = pid.GetKp();
    double Ki = pid.GetKi();
    double Kd = pid.GetKd();

    // Use the optimized PID values in your PID controller
    // (You may need to scale them to the appropriate range for your system)
  }
}
