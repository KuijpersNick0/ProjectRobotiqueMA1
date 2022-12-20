#include <PID_v1.h> 

#define TURN_DURATION 650  // Turn duration in milliseconds
#define TURN_SPEED 255  // Turn speed (0-255)

#define TURN_INTERVAL 8000   

bool turnRight = false;

unsigned long start_time;  // Variable to store the start time
 
#define Kp 0.96 
#define Ki 0.5
#define Kd 0.00

double setpoint1 = 170.0;
double setpoint2 = 170.0;
double input1 = 0.0;
double input2 = 0.0; 
double output1, output2;

double test=0;

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

//last time each encopder was triggered
volatile long int lastG = 0;
volatile long int lastD = 0;

//previous value of the integral part of the pi regulator
double I_lastG = 0;
double I_lastD = 0;  

//Struct containing the data of the left motor
struct {
  bool dir = 1;             //Direction of the motor
  volatile int Count = 0;   //Number of pulses received from the encoder since the last reset
  volatile int Count_T = 0; //Total number of pulses received from the encoder since the beginning of the program
  float Speed = 0;          //Speed command sent to the motor
  float MSpeed = 0;         //Measured speed of the motor
} motG;

//Struct containing the data of the left motor
struct {
  bool dir = 1;             //Direction of the motor
  volatile int Count = 0;   //Number of pulses received from the encoder since the last reset
  volatile int Count_T = 0; //Total number of pulses received from the encoder since the beginning of the program
  float Speed = 0;          //Speed command sent to the motor
  float MSpeed = 0;         //Measured speed of the motor
} motD;


double alpha = 0.05; // Adjust this value to control the filter strength
double filtered_input1 = 0.0; 
 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  
  pinMode(pwmPinD, OUTPUT);
  pinMode(pwmPinG, OUTPUT);

  pinMode(encoder_G, INPUT);// setting up encoder pin as input
  pinMode(encoder_D, INPUT); 
 //attach the interupts to the pins
  attachInterrupt(digitalPinToInterrupt(encoder_G), EncodeurG, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_D), EncodeurD, CHANGE);

   // Initialize the setpoint and input variables
  setpoint1 = 170;
  setpoint2 = 170; 
 
  myPID_Left.SetOutputLimits(0,200);
  myPID_Left.SetMode(AUTOMATIC);
  
  myPID_Right.SetOutputLimits(0,200);
  myPID_Right.SetMode(AUTOMATIC);

  start_time = millis();
}

void loop() { 
  setpoint1 = 170;
  setpoint2 = 170;
  
  // Check if the delay interval has elapsed
  if (millis() - start_time >= TURN_INTERVAL) {   
    setpoint1 = 120;
    input1 = motD.MSpeed;
    myPID_Left.Compute(); 
    output1 = map_range(output1, 0, 200, 0, 255);  
    
    setpoint2 = 20;
    input2 = motD.MSpeed; 
    myPID_Right.Compute();
    output2 = map_range(output2, 0, 200, 0, 255);   

    tourneDroite(output1, output2);
      
    delay(TURN_DURATION); 
    stopTurning();
    delay(100);
    start_time = millis();
  }
  setpoint1 = 170;
  setpoint2 = 170;
  
  //avanceGauche(120); 
  //avanceDroite(120);
  getspeed();
  input1 = motG.MSpeed; 
  //Serial.println(input1);
  myPID_Left.Compute(); 
  //Serial.println(input1);
  output1 = map_range(output1, 0, 200, 0, 255); 
  avanceGauche(output1); 
  //Serial.println(output1);  
  input2 = motD.MSpeed;
  //Serial.println(input2); 
  myPID_Right.Compute();
  output2 = map_range(output2, 0, 200, 0, 255); 
  avanceDroite(output2);
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

//Function linked to the interupt pin connected to the left encorder
void EncodeurG() {
  if (motG.dir) {                 //If the motor is going forward    
    if (millis() - lastG > 10) {  //If the last pulse was received more than 10ms ago (debounce)
      motG.Count++;
      motG.Count_T++;
      lastG = millis();           //Update the last time a pulse was received
    } 
  } else {
    if (millis() - lastG > 10) {  //If the last pulse was received more than 10ms ago (debounce)
      motG.Count--;
      motG.Count_T--;
      lastG = millis();           //Update the last time a pulse was received
    }
  }
}


//Function linked to the interupt pin connected to the right encorder
void EncodeurD() {
  if (motD.dir) {                 //If the motor is going forward
    if (millis() - lastD > 10) {  //If the last pulse was received more than 10ms ago (debounce)
      motD.Count++;
      motD.Count_T++;
      lastD = millis();           //Update the last time a pulse was received
    }
  } else {
    if (millis() - lastD > 10) {  //If the last pulse was received more than 10ms ago (debounce)
      motD.Count--;
      motD.Count_T--;
      lastD = millis();           //Update the last time a pulse was received
    }
  }
}

double lowPassFilter(double input1) {
  // Calculate the filtered input for motor 1
  filtered_input1 = alpha * input1 + (1.0 - alpha) * filtered_input1; 
  return filtered_input1;
} 

//Function to get the speed of the motors
void getspeed() {
  motG.Count = 0;   //resets the number of pulses received from the encoders
  motD.Count = 0;
  delay(200);       //counts the number of pulses received during 200ms

  motG.MSpeed = motG.Count * 5 * 5.1;   //calculates the speed in mm per second (*5 to get to 1s  *5.1 as 1 pulse equals to 5.1 mm)
  motD.MSpeed = motD.Count * 5 * 5.1;
  motG.MSpeed = lowPassFilter(motG.MSpeed);
  motD.MSpeed = lowPassFilter(motD.MSpeed);  
  //Serial.println(motG.MSpeed);
}

int map_range(int input_value, int input_min, int input_max, int output_min, int output_max) {
  // Calculate the input value as a percentage of the input range
  int input_range = input_max - input_min;
  int input_percentage = (input_value - input_min) * 100 / input_range;

  // Calculate the output value using the input percentage and the output range
  int output_range = output_max - output_min;
  int output_value = (input_percentage * output_range) / 100 + output_min;

  return output_value;
} 

void stopTurning() {
  // Stop the motors
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin1, LOW); 
}

void tourneDroite(int val1, int val2){  
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH); 
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  analogWrite(pwmPinG, val1);
  analogWrite(pwmPinD, val2);
}
