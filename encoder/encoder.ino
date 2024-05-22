#include <Encoder.h>

//Angulos 
int theta_deseada = 0; int theta_error = 0; float theta_actual;

//errores
float error; int error_ante = 0;

//Constantes de control
const float kp = 1; const float ki = 0.025; const float kd = 0.001; 

//PID
float Deriv; float Propo; float Integ; float PID;

//Variables extra
int integral = 0; int pulso_encoder = 0; int mapeo=0; 

//IO
int vel = 5; int Left = 10;   int Right = 12; 

Encoder myEnc(2, 3); //Pines Encoder

void setup() {
  Serial.begin(9600);
  pinMode(vel,OUTPUT);
  pinMode(Right,OUTPUT);
  pinMode(Left,OUTPUT);
}

long oldPosition  = -999;


void Read() {
  // Esperamos datos
  if (Serial.available() > 0) {
    // Leer valor
    theta_deseada = Serial.parseInt();
    // Esperar ENTER de usuario
    while (Serial.available() > 0) {
      Serial.read(); // Limpiamos el buffer
      delay(2); 
    }
    Serial.print("Theta deseada: ");
    Serial.println(theta_deseada);
  }
}


void control(){
  error=theta_deseada-theta_actual;
  if (error > 1){
    digitalWrite(Left,LOW);
    digitalWrite(Right,HIGH);
  } else if (error<-1){
    digitalWrite(Left,HIGH);
    digitalWrite(Right,LOW);
  } else {
    digitalWrite(Left,LOW);
    digitalWrite(Right,LOW);
  }
  
  Deriv =  (kd*(error-error_ante));
  Propo = (kp*error);
  Integ = (ki*(integral));
  PID=Propo+Deriv+Integ; 
    
}


void loop() {
  Read();
  long newPosition = myEnc.read();
  theta_actual= newPosition*0.1208;
  Serial.println(theta_actual);
  control();
  Serial.print("Pulsos de Encoder");
  Serial.println(newPosition);
  Serial.print("Error:");
  Serial.println(error);
  Serial.print("salida de control:");
  if (PID > 0){
    mapeo = map(PID,0,1023,100,255);
  } else {
    mapeo = map(PID,0,-1023,100,255);
  }

  Serial.println(mapeo);
  analogWrite(vel,mapeo);

  integral+= error;
  delay(100);

}