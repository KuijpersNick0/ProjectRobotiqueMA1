#include <Servo.h>

Servo monServo;  // on crée l'objet monServo

// defines pins numbers capteur ultrason 
const int trigPinDroite = 5;
const int echoPinDroite = 6;
const int trigPinCentre = 2;
const int echoPinCentre = 3;

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

//Capteur de proximité 
int capteurProximite = 4;
int detectionProximite; 


void setup() { 
  pinMode(trigPinDroite, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinDroite, INPUT); // Sets the echoPin as an Input
  pinMode(trigPinCentre, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinCentre, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication

  // put your setup code here, to run once:
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  //Capteur proximite
  pinMode(capteurProximite, INPUT);
}
void loop() {
  // Clears the trigPin
  digitalWrite(trigPinDroite, LOW);
  digitalWrite(trigPinCentre, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPinDroite, HIGH);
  digitalWrite(trigPinCentre, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinDroite, LOW);
  digitalWrite(trigPinCentre, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationEchoDroite = pulseIn(echoPinDroite, HIGH);
  durationEchoCentre = pulseIn(echoPinCentre, HIGH);
  // Calculating the distance
  distanceDroite = durationEchoDroite * 0.034 / 2;
  distanceCentre = durationEchoCentre * 0.034 / 2;
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
    Serial.print("STTTTTTTOP");
  }
  else {
    if (distanceCentre >= 5){
      // capteur distance est + grande que 5   
      digitalWrite(motor1pin1, HIGH);
      digitalWrite(motor1pin2, HIGH);
    
      digitalWrite(motor2pin1, HIGH);
      digitalWrite(motor2pin2, HIGH);
      delay(1000);
    
      digitalWrite(motor1pin1, LOW);
      digitalWrite(motor1pin2, HIGH);
    
      digitalWrite(motor2pin1, LOW);
      digitalWrite(motor2pin2, HIGH);
      delay(1000);
    }
  } 
}
