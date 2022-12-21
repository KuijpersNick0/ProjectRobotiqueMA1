// Declare variables for the PID control

double setpoint1; // The setpoint for the first motor
double input1; // The current speed of the first motor
double output1; // The output of the PID controller for the first motor

double setpoint2; // The setpoint for the second motor
double input2; // The current speed of the second motor
double output2; // The output of the PID controller for the second motor

double Kp = 4; // The proportional gain
double Ki = 0.2; // The integral gain
double Kd = 1; // The derivative gain

double errorSum1 = 0; // The sum of errors for integral calculation for the first motor
double errorSum2 = 0; // The sum of errors for integral calculation for the second motor
double lastError1 = 0; // The error in the previous time step for the first motor
double lastError2 = 0; // The error in the previous time step for the second motor

double dt = 1; // The time step for the PID control (in seconds)

//Define all the pins used for the interupts
#define intG 2
#define intD 3
#define intProxi 0

int motG_Count=0;
int motD_Count=0;
volatile long int lastG = 0;
volatile long int lastD = 0;


// Declare the time step for the speed calculation (in seconds)
const double dtE = 0.1;

// Declare the number of encoder ticks per revolution
const int ticksPerRev = 16;

// Declare the radius of the encoder wheel (in meters)
const double wheelRadius = 0.1;

// Declare the previous encoder tick count
long previousTicks = 0;

// Declare the previous time
unsigned long previousTime = 0;

//Moteur
int motor1pin1 = 5;
int motor1pin2 = 4; 
int motor2pin1 = 6;
int motor2pin2 = 7; 
int pwmPinD = 10;
int pwmPinG = 11;


void setup() {
  // Initialize the setpoint and input variables
  setpoint1 = 50;
  setpoint2 = 50;
  input1 = getSpeedG();
  input2 = getSpeedD();

  // Initialize the output to 0
  output1 = 0;
  output2 = 0;

  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(intG), EncodeurG, CHANGE);
  attachInterrupt(digitalPinToInterrupt(intD), EncodeurD, CHANGE);

  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  
  pinMode(pwmPinD, OUTPUT);
  pinMode(pwmPinG, OUTPUT);
  
}

void loop() {
  // Calculate the error for the first motor
  double error1 = setpoint1 - input1;

  // Calculate the integral term for the first motor
  errorSum1 += error1 * dt;

  // Calculate the derivative term for the first motor
  double dError1 = (error1 - lastError1) / dt;

  // Calculate the output of the PID controller for the first motor
  output1 = Kp * error1 + Ki * errorSum1 + Kd * dError1;

  // Calculate the error for the second motor
  double error2 = setpoint2 - input2;

  // Calculate the integral term for the second motor
  errorSum2 += error2 * dt;

  // Calculate the derivative term for the second motor
  double dError2 = (error2 - lastError2) / dt;

  // Calculate the output of the PID controller for the second motor
  output2 = Kp * error2 + Ki * errorSum2 + Kd * dError2;

  // Update the input and last error variables
  input1 = getSpeedG();
  input2 = getSpeedD();
  lastError1 = error1;
  lastError2 = error2;

  // Use the output to control the motors
  setMotor1(output1);
  setMotor2(output2);

  //AVEC ECRITURE 2
  //setMotor(pwmPin1, speed1);
  //setMotor(pwmPin2, speed2);  
  Serial.println(input1 );
  Serial.println("okay");
  Serial.println(input2 );
  Serial.println("Nope");
  // Wait for the next time ste
  motG_Count = 0;
  motD_Count = 0;
  delay(dt * 1000);
}


void setMotor1(int speed) {
  // Convert the speed from a percentage (0-100) to a duty cycle (0-255)
  int dutyCycle = map(speed, 0, 100, 0, 255);

  // Set the PWM duty cycle on the pin
  analogWrite(pwmPinD, dutyCycle);
}

void setMotor2(int speed) {
  // Convert the speed from a percentage (0-100) to a duty cycle (0-255)
  int dutyCycle = map(speed, 0, 100, 0, 255);

  // Set the PWM duty cycle on the pin
  analogWrite(pwmPinG, dutyCycle);
}


double getSpeedG() {
  // Read the current encoder tick count
  long currentTicks = motG_Count;

  // Read the current time
  unsigned long currentTime = millis();

  // Calculate the number of ticks since the previous measurement
  long tickDelta = currentTicks - previousTicks;

  // Calculate the time since the previous measurement
  double timeDelta = (currentTime - previousTime) / 1000.0;

  // Calculate the speed in RPM
  double rpm = (tickDelta / ticksPerRev) / (timeDelta / 60.0);

  // Calculate the speed in m/s
  double speed = (rpm / 60.0) * 2 * PI * wheelRadius;

  // Update the previous tick count and time
  previousTicks = currentTicks;
  previousTime = currentTime;

  return speed;
}


double getSpeedD() {
  // Read the current encoder tick count
  long currentTicks = motD_Count;

  // Read the current time
  unsigned long currentTime = millis();

  // Calculate the number of ticks since the previous measurement
  long tickDelta = currentTicks - previousTicks;

  // Calculate the time since the previous measurement
  double timeDelta = (currentTime - previousTime) / 1000.0;

  // Calculate the speed in RPM
  double rpm = (tickDelta / ticksPerRev) / (timeDelta / 60.0);

  // Calculate the speed in m/s
  double speed = (rpm / 60.0) * 2 * PI * wheelRadius;

  // Update the previous tick count and time
  previousTicks = currentTicks;
  previousTime = currentTime;

  return speed;
}


void EncodeurG() {           
  if (millis() - lastG > 10) {  //If the last pulse was received more than 10ms ago (debounce)
    motG_Count++;
    lastG = millis();           //Update the last time a pulse was received
  }
}

//Function linked to the interupt pin connected to the right encorder
void EncodeurD() {           
  if (millis() - lastD > 10) {  //If the last pulse was received more than 10ms ago (debounce)
    motD_Count++;
    lastD = millis();           //Update the last time a pulse was received
  }
}
