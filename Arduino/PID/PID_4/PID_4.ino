#include <PID_v1.h>

//Kp = 1.6*0.60
//1.6 == systeme oscillatoire
#define Kp 0.96 
#define Ki 0.5 
#define Kd 0.01

double setpoint1 = 50.0;
double setpoint2 = 50.0;
double input1 = 0.0;
double input2 = 0.0; 
double output1, output2;

PID myPID_Left(&input1, &output1, &setpoint1, Kp, Ki, Kd, DIRECT);
PID myPID_Right(&input2, &output2, &setpoint2, Kp, Ki, Kd, DIRECT);

//Moteur
int motor1pin1 = 5;
int motor1pin2 = 4; 
int motor2pin1 = 6;
int motor2pin2 = 7; 
int pwmPinD = 11;
int pwmPinG = 10;

#define encoder_G 3
#define encoder_D 2
 

unsigned int rpm_G;
volatile byte pulses_G;
unsigned long TIME_G;
unsigned int pulse_per_turn_G = 20; 
//depends on the number of slots on the slotted disc
unsigned int rpm_D;
volatile byte pulses_D;
unsigned long TIME_D;
unsigned int pulse_per_turn_D = 3; 
//depends on the number of slots on the slotted disc

double alpha = 0.1; // Adjust this value to control the filter strength
double filtered_input1 = 0.0; 
 
void countG(){
  // counting the number of pulses for calculation of rpm
  pulses_G++;
}
void countD(){
  // counting the number of pulses for calculation of rpm
  pulses_D++;
} 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //reset all to 0
  rpm_G = 0;
  pulses_G = 0;
  TIME_G = 0;
  rpm_D = 0;
  pulses_D = 0;
  TIME_D = 0;

   // Initialize the setpoint and input variables
  setpoint1 = 120;
  setpoint2 = 120; 
 
  myPID_Left.SetOutputLimits(0,255);
  myPID_Left.SetMode(AUTOMATIC);
  
  myPID_Right.SetOutputLimits(0,255);
  myPID_Right.SetMode(AUTOMATIC);

  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  
  pinMode(pwmPinD, OUTPUT);
  pinMode(pwmPinG, OUTPUT);

  pinMode(encoder_G, INPUT);// setting up encoder pin as input
  pinMode(encoder_D, INPUT);
  //triggering count function everytime the encoder turns from HIGH to LOW
  attachInterrupt(digitalPinToInterrupt(encoder_G), countG, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoder_D), countD, FALLING); 
}

void loop() {
  // put your main code here, to run repeatedly:

  calculateRPM_G(); 
  calculateRPM_D(); 
    
  //Serial.println("rpms : ");
  //Serial.println(rpm_G); 
  //Serial.println(rpm_D); 
  
  input1 = rpmToPwmG(rpm_G);
  input2 = rpmToPwmD(rpm_D);

  avanceGauche(120);

  Serial.println(pulses_G);

  //input1 = lowPassFilter(input1);
  //input2 = lowPassFilter(input2);

  //Serial.println("inputs : ");
  //Serial.println(input1);
  //Serial.print("\t");
  //Serial.println(input2);  

  // calculate the output value
  myPID_Left.Compute();
  myPID_Right.Compute();

  //Serial.println("outputs : ");
  //Serial.println(output1);
  //Serial.println(output2);  

  //Serial.println(output1);
  //Serial.print("\t");
  //Serial.println(output2);
  
  //setMotorSpeed(pwmPinG, motor1pin1, motor1pin2, output1);
  //setMotorSpeed(pwmPinD, motor2pin1, motor2pin2, output2); 
}

void setMotorSpeed(int pwmPin, int motorpin1, int motorpin2, int speed) {
  // set the direction of the motor
  if (speed >= 0) {
    digitalWrite(motorpin1, LOW);
    digitalWrite(motorpin2, HIGH);
  } else {
    digitalWrite(motorpin1, HIGH);
    digitalWrite(motorpin2, LOW);
  }
  // set the speed of the motor using PWM
  analogWrite(pwmPin, abs(speed));
}

void calculateRPM_G(){
  if (millis() - TIME_G >= 100){ // updating every 0.1 second
    detachInterrupt(digitalPinToInterrupt(encoder_G)); // turn off trigger
    //calcuate for rpm 
    rpm_G = (60 *100 / pulse_per_turn_G)/ (millis() - TIME_G) * pulses_G;
    TIME_G = millis();
    pulses_G = 0;
    //print output 
    //Serial.print("millis: ");
    //Serial.println(millis()-TIME_G);
    //trigger count function everytime the encoder turns from HIGH to LOW
    attachInterrupt(digitalPinToInterrupt(encoder_G), countG, FALLING);
  }
}

void calculateRPM_D(){
  if (millis() - TIME_D >= 100){ // updating every 0.1 second
    detachInterrupt(digitalPinToInterrupt(encoder_D)); // turn off trigger
    //calcuate for rpm 
    rpm_D = (60 *100 / pulse_per_turn_D)/ (millis() - TIME_D) * pulses_D;
    TIME_D = millis();
    pulses_D = 0;
    //print output 
    //Serial.print("RPM D: ");
    //Serial.println(rpm_D);
    //trigger count function everytime the encoder turns from HIGH to LOW
    attachInterrupt(digitalPinToInterrupt(encoder_D), countD, FALLING);
  }
}


int rpmToPwmG(float rpm) {
  // Calculate the duty cycle required to achieve the desired RPM
  float dutyCycle = (rpm / 60.0) * 255.0 / 2; 
  return  (int) dutyCycle;   
}

int rpmToPwmD(float rpm) {
  // Calculate the duty cycle required to achieve the desired RPM
  float dutyCycle = (rpm / 60.0) * 255.0 / 2; 
  return (int) dutyCycle; 
}

double lowPassFilter(double input1) {
  // Calculate the filtered input for motor 1
  filtered_input1 = alpha * input1 + (1.0 - alpha) * filtered_input1; 
  return filtered_input1;
} 

//Fonction moteur avance
void avanceGauche(int val){
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);  
  analogWrite(pwmPinG,val); 
}

void avanceDroite(int val){
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);  
  analogWrite(pwmPinD,val); 
}
