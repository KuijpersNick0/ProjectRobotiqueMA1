#include <Servo.h> 

Servo monServo;  // on crée l'objet monServo

// defines pins numbers capteur ultrason 
const int trigPinDroite = 5;
const int echoPinDroite = 7;
const int trigPinCentre = 12;
const int echoPinCentre = 13;

// defines variables
long durationEchoDroite;
int distanceDroite;
long durationEchoCentre;
int distanceCentre;

//Moteur
int motor1pin1 = 11;
int motor1pin2 = 10; 
int motor2pin1 = 8;
int motor2pin2 = 9; 
int pwmMotor1 = 3;
int pwmMotor2 = 6;

 
//Capteur de proximité 
int capteurProximite = 4;
int detectionProximite;  

//Fonction capteur infrarouge
long measure(int trigger , int echo);

//Fonctions moteur
void avance(int val);
void tourneDroite(); 
void tourneGauche();

bool test = true;

void setup() { 
  //capt infrarouge
  pinMode(trigPinDroite, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinDroite, INPUT); // Sets the echoPin as an Input
  pinMode(trigPinCentre, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinCentre, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication

  //Moteur et PWM
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(pwmMotor1, OUTPUT);
  pinMode(pwmMotor2, OUTPUT);
  
  //Capteur proximite
  pinMode(capteurProximite, INPUT);
}
void loop() { 
  // Calculating the distance
  distanceDroite = measure(trigPinDroite, echoPinDroite);
  distanceCentre = measure(trigPinCentre, echoPinCentre);
  // Prints the distance on the Serial Monitor
  Serial.print("DistanceDroite: ");
  Serial.println(distanceDroite);
  Serial.println(distanceDroite/100); 

  Serial.print("DistanceCentre: ");
  Serial.println(distanceCentre);
  Serial.println(distanceCentre/100); 
   
  // Lecture de la valeur de l'interface OUT du capteur
  detectionProximite = digitalRead(capteurProximite);

  // Si on détecte un bord, on STOP WESH
  if (detectionProximite == 0) {
    Serial.print("STOP DE CAPT PROXIMITE");
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, LOW); 
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, LOW);
  }
  else {
     
    if ( distanceDroite >= 10 ){  
      if (test){
        avance(100);
        delay(100);
        test=false;
      }
      tourneDroite(110); 
      Serial.print("je tourne droite"); 
    }
    else if (distanceCentre >= 10) {
      avance(110); 
      Serial.print("j'avance"); 
      test =true;
    }
    else if (distanceCentre < 10 && distanceDroite < 10){
      tourneGauche(110);
      Serial.print("Je tourne gauche");
    }
    else {
      Serial.print("STOP DE CAPT INFRAROUGE");
      digitalWrite(motor1pin1, LOW);
      digitalWrite(motor1pin2, LOW);     
      digitalWrite(motor2pin1, LOW);
      digitalWrite(motor2pin2, LOW);
    }
  } 
}


//Fonction mesure distance
long measure(int trigger, int echo){
  long duration = 0;
  digitalWrite(trigger, LOW);
  delay(2);
  digitalWrite(trigger, HIGH);
  delay(10);
  digitalWrite(trigger, LOW);
  duration = pulseIn(echo, HIGH, 11662);
  return (duration/2)*0.03432;
}

//Fonction moteur avance
void avance(int val){
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH); 
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
  analogWrite(pwmMotor1,val);
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
