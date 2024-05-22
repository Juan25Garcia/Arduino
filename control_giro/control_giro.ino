#include <Encoder.h>
#include <math.h>

// Angulos 
float theta_deseada = 0; 
float theta_actual;

//coordenadas
const int x = 0;
const int y = 0;
const int xd = 0;
const int yd = 1;

// Errores
float error; 
float error_ante = 0;

// Constantes de control
const float kp = 2; 
const float ki = 0.025; 
const float kd = 0.001; 

// PID
float Deriv; 
float Propo; 
float Integ; 
float PID;

// Variables extra
int integral = 0; 
int pulso_encoder = 0; 
int mapeo = 0; 

// IO
int vel = 6; 
int Left = 8;    
int Right = 9; 

//int vel2 = 5
//int Left2 = 10;    
//int Right2 = 12; 

Encoder myEnc(2, 3); // Pines Encoder
//Encoder2 myEnc(0, 1); // Pines Encoder

void setup() {
  Serial.begin(9600);
  pinMode(vel, OUTPUT);
  pinMode(Right, OUTPUT);
  pinMode(Left, OUTPUT);
}

long oldPosition = -999;

void Read() {
  // Esperamos datos
    theta_deseada = atan2(yd - y, xd - x);
    theta_deseada = theta_deseada* (180.0 / PI);
    Serial.print("Theta deseada: ");
    Serial.println(theta_deseada);
}

void control() {
  error = theta_deseada - theta_actual;

  if (error > 1) {
    digitalWrite(Left, LOW);
    digitalWrite(Right, HIGH);
  } else if (error < -1) {
    digitalWrite(Left, HIGH);
    digitalWrite(Right, LOW);
  } else {
    digitalWrite(Left, LOW);
    digitalWrite(Right, LOW);
  }
  
  // PID calculations
  Deriv = kd * (error - error_ante);
  Propo = kp * error;
  Integ = ki * integral;
  PID = Propo + Deriv + Integ;

  // Limit the integral to prevent windup
  if (Integ > 255) {
    Integ = 255;
  } else if (Integ < -255) {
    Integ = -255;
  }
  
  // Update previous error
  error_ante = error;
}

void loop() {
  Read();
  long newPosition = myEnc.read();
  theta_actual = newPosition * 0.1208; // Asumiendo un factor de escala
  Serial.println(theta_actual);
  
  control();

  Serial.print("Pulsos de Encoder: ");
  Serial.println(newPosition);
  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("Salida de control: ");

  if (PID > 0) {
    mapeo = map(PID, 0, 1023, 100, 255);
  } else {
    mapeo = map(PID, 0, -1023, 100, 255);
  }

  Serial.println(mapeo);
  analogWrite(vel, mapeo);

  integral += error;
}
