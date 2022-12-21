#define Kp 0.96
#define Ki 200
#define Kd 0.01

double setpoint1 = 50.0;
double setpoint2 = 50.0;
double input1 = 0.0;
double input2 = 0.0;
double error1, error2, previous_error1, previous_error2;
double integral1, integral2, derivative1, derivative2;
double output1, output2;
unsigned long previous_time, current_time;

//Moteur
int motor1pin1 = 5;
int motor1pin2 = 4; 
int motor2pin1 = 6;
int motor2pin2 = 7; 
int pwmPinD = 11;
int pwmPinG = 10;

#define encoder_G 3
#define encoder_D 2

int INTEGRAL_WINDUP_LIMIT = 30; 

unsigned int rpm_G;
volatile byte pulses_G;
unsigned long TIME_G;
unsigned int pulse_per_turn_G = 3; 
//depends on the number of slots on the slotted disc
unsigned int rpm_D;
volatile byte pulses_D;
unsigned long TIME_D;
unsigned int pulse_per_turn_D = 3; 
//depends on the number of slots on the slotted disc

bool arriereG = false;
bool arriereD = false;

void countG(){
  // counting the number of pulses for calculation of rpm
  pulses_G++;
}
void countD(){
  // counting the number of pulses for calculation of rpm
  pulses_D++;
}



double alpha = 0.1; // Adjust this value to control the filter strength
double filtered_input1 = 0.0; 
 
void setup() {
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
  input1 = 0;
  input2 = 0;

  // Initialize the output to 0
  output1 = 0;
  output2 = 0;


  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  
  pinMode(pwmPinD, OUTPUT);
  pinMode(pwmPinG, OUTPUT);

  pinMode(encoder_G, INPUT);// setting up encoder pin as input
  pinMode(encoder_D, INPUT);
  //triggering count function everytime the encoder turns from HIGH to LOW
  attachInterrupt(digitalPinToInterrupt(encoder_G), countG, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_D), countD, CHANGE);
}

void loop() {
  calculateRPM_G(); 
  calculateRPM_D();

  input1 = rpmToPwmG(rpm_G);
  input2 = rpmToPwmD(rpm_D);

  //Serial.println("inputs : ");
  //Serial.println(input1);
  //Serial.println(input2);  

  //input1 = lowPassFilter(input1);
  //input2 = lowPassFilter(input2); 

  //Serial.println("inputs modified : ");
  //Serial.println(input1);
  //Serial.println(input2);  
  
  //Serial.println("setpoint : ");
  //Serial.println(setpoint1);
  //Serial.println(setpoint2);
  
  // Calculate errors
  error1 = setpoint1 - input1;
  error2 = setpoint2 - input2;

  // Calculate elapsed time
  current_time = millis();
  double elapsed_time = (current_time - previous_time) / 1000.0;

  // Calculate integrals
  integral1 += error1 * elapsed_time;
  integral2 += error2 * elapsed_time;

  //Anti-windup 
  integral1 = clamp(integral1, 0, INTEGRAL_WINDUP_LIMIT);
  integral2 = clamp(integral2, 0, INTEGRAL_WINDUP_LIMIT);

  // Calculate derivatives
  derivative1 = (error1 - previous_error1) / elapsed_time;
  derivative2 = (error2 -  previous_error2) / elapsed_time;
  
  //Serial.println("derivative : ");
  //Serial.println(derivative1);
  //Serial.println(derivative2);

  //Serial.println("integral : ");
  //Serial.println(integral1);
  //Serial.println(integral2);

  //Serial.println("proportional : ");
  //Serial.println(Kp*error1);
  //Serial.println(Kp*error2);
  
  
  // Calculate outputs
  output1 = Kp * error1 + Ki * integral1 + Kd * derivative1;
  output2 = Kp * error2 + Ki * integral2 + Kd * derivative2;

  output1 = Kp * error1 ;  
  output2 = Kp * error2 ;  

    

  output1 = constrain(output1, 0, 255);
  output2 = constrain(output2, 0, 255);
  // Use outputs to adjust motors 
  
  //Serial.println("outputs : ");
  Serial.println(output1);
  //Serial.println(output2); 
  
  avanceGauche(output1);
  avanceDroite(output2);
  
 
  
  // Store current errors and time for next iteration
  previous_error1 = error1;
  previous_error2 = error2;
  previous_time = current_time;


  delay(1000);
 
}



//Fonction moteur avance
void avanceGauche(int val){
  if (val >= 0) {
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
    analogWrite(pwmPinG,val); 
    arriereG = false;
  } else {
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
    analogWrite(pwmPinG,val);
    arriereG = true; 
  }
}

void avanceDroite(int val){
  if (val >= 0) {
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
    analogWrite(pwmPinD,val); 
    arriereD = false;
  } else {
    digitalWrite(motor2pin1, HIGH);
    digitalWrite(motor2pin2, LOW);
    analogWrite(pwmPinD,val); 
    arriereD = true;
  } 
}


void calculateRPM_G(){
  if (millis() - TIME_G >= 100){ // updating every 0.1 second
    detachInterrupt(digitalPinToInterrupt(encoder_G)); // turn off trigger
    //calcuate for rpm 
    rpm_G = (60 *100 / pulse_per_turn_G)/ (millis() - TIME_G) * pulses_G;
    TIME_G = millis();
    pulses_G = 0;
    //print output 
    //Serial.print("RPM G: ");
    //Serial.println(rpm_G);
    //trigger count function everytime the encoder turns from HIGH to LOW
    attachInterrupt(digitalPinToInterrupt(encoder_G), countG, CHANGE);
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
    attachInterrupt(digitalPinToInterrupt(encoder_D), countD, CHANGE);
  }
}


int rpmToPwmG(float rpm) {
  // Calculate the duty cycle required to achieve the desired RPM
  float dutyCycle = (rpm / 60.0) * 255.0 / 2;

  // Return the duty cycle as an integer
  if (arriereG ==true){
    return -(int) dutyCycle;  
  } else {
    return (int) dutyCycle;
  }  
}

int rpmToPwmD(float rpm) {
  // Calculate the duty cycle required to achieve the desired RPM
  float dutyCycle = (rpm / 60.0) * 255.0 / 2;

  // Return the duty cycle as an integer
  if (arriereD==true){
    return -(int) dutyCycle;  
  } else {
    return (int) dutyCycle;
  }
}


// Windup guard for integral term
float clamp(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}


double lowPassFilter(double input1) {
  // Calculate the filtered input for motor 1
  filtered_input1 = alpha * input1 + (1.0 - alpha) * filtered_input1; 
  return filtered_input1;
} 
