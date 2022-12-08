//Define all the pins used for the interupts
#define intG 2
#define intD 3
#define intProxi 0

int motG_Count=0;
int motD_Count=0;
//last time each encopder was triggered
volatile long int lastG = 0;
volatile long int lastD = 0;

//Moteur
int motor1pin1 = 5;
int motor1pin2 = 4; 
int motor2pin1 = 6;
int motor2pin2 = 7; 
int pwmMotor1 = 10;
int pwmMotor2 = 11;

int speedG = 0;
int speedD = 0;

//Fonctions moteur
void avance(int val);
void tourneDroite(); 
void tourneGauche();

int TARGET = 204*0.75 ;
int Kp = 0.02;
int e1_error = 0;
int e2_error = 0;
 
void setup() {
  
    //attach the interupts to the pins
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(intG), EncodeurG, CHANGE);
  attachInterrupt(digitalPinToInterrupt(intD), EncodeurD, CHANGE);


  //Moteur et PWM
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(pwmMotor1, OUTPUT);
  pinMode(pwmMotor2, OUTPUT);


}

void loop() {
  //Serial.println("oui");
  //Serial.println(motG_Count);
  //Serial.println(motD_Count);
  //delay(100);
  
   // put your main code here, to run repeatedly:

  while (true){
    //Serial.println(motG_Count);
    //Serial.println(motD_Count);
    
    delay(100);
     
    getSpeed();
     
    avanceGauche(80);
    avanceDroite(80); 

    e1_error = TARGET - speedG;
    e2_error = TARGET - speedD; 
    //Serial.println(e1_error);
    //Serial.println(e2_error);
  
    valInter += Kp*e1_error;
    valInter += Kp*e2_error; 

    
    
  }
  
  
}

//Function linked to the interupt pin connected to the left encorder
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


//Fonction moteur avance
void avanceGauche(int val){
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);  
  analogWrite(pwmMotor1,val); 
}

void avanceDroite(int val){
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);  
  analogWrite(pwmMotor2,val); 
}
 
void tourneDroite(int val){ 
  
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW); 
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
  analogWrite(pwmMotor1,val);
  analogWrite(pwmMotor2,val);
}

void tourneGauche(int val){
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH); 
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
  analogWrite(pwmMotor1,val);
  analogWrite(pwmMotor2,val);
}

void getSpeed() {
  motG_Count = 0;
  motD_Count = 0;
  delay(200);
  //Serial.println("SPEEED" );
  speedG = motG_Count*5*5.1;
  speedD = motD_Count*5*5.1;
  //Serial.println(speedG);
  //Serial.println(speedD);
}

void recule(int val){
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW); 
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  analogWrite(pwmMotor1,val);
  analogWrite(pwmMotor2,val);
}
