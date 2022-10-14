int motorPin1 = 12;
int motorPin2 = 11;
int enablePin = 13;
 
// Vitesse du moteur
int state = 0;
void setup() {
    // Configuration des ports en mode "sortie"
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(enablePin, OUTPUT);
    
    // Initialisation du port série
    Serial.begin(9600);
}
 
void loop() {
    if (Serial.available() &gt; 0)
    {
      // Lecture de l'entier passé au port série
      state = Serial.parseInt();
      //
      // Sens du mouvement
      //
      if (state &gt; 0) // avant
      {
        digitalWrite(motorPin1, HIGH); 
        digitalWrite(motorPin2, LOW);
        Serial.print("Avant ");
        Serial.println(state);
      }
      else if (state &lt; 0) // arrière
      {
        digitalWrite(motorPin1, LOW); 
        digitalWrite(motorPin2, HIGH);
        Serial.print("Arriere ");
        Serial.println(state);
      }
      else // Stop (freinage)
      {
        digitalWrite(motorPin1, HIGH); 
        digitalWrite(motorPin2, HIGH);
        Serial.println("Stop");
      }
      //
      // Vitesse du mouvement
      //
      analogWrite(enablePin, abs(state));
    }
    delay(100);
}
