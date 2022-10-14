const int IN_D0_1 = 2; // digital input 
int encodeur_cnt = 0; 

//Moteur
int motor1pin1 = 11;
int motor1pin2 = 10;

int motor2pin1 = 8;
int motor2pin2 = 9;

void setup() {

  // put your setup code here, to run once:

  pinMode (IN_D0_1, INPUT); 
  
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT); 
  Serial.begin(9600); 
}



void loop() {

  // put your main code here, to run repeatedly:
 
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, HIGH); 
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, HIGH);

  int value = digitalRead(IN_D0_1); // reads the analog input from the IR distance sensor 
  
  if (value == 1){ 
    encodeur_cnt +=1;
    Serial.println(encodeur_cnt);
    if (encodeur_cnt==20){
      encodeur_cnt = 0;
      digitalWrite(motor1pin1, LOW);
      digitalWrite(motor1pin2, LOW); 
      digitalWrite(motor2pin1, LOW);
      digitalWrite(motor2pin2, LOW);
      delay(1000); 
    } 
  }  
}
