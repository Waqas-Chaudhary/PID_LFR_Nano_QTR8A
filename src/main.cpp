#include<Arduino.h>
#include <QTRSensors.h>
QTRSensors QTR;
uint16_t Sensor_Readings[8];
uint16_t Position = 0;
const uint8_t sensors = 8;
int E_pin = 7;  //Emitter pin for QTR8A IR Array

//MX1508 Motor Driver Pins
int IN1 = 10;
int IN2 = 11;
int IN3 = 6;
int IN4 = 9;


//Speeds
int SpeedL = 0; //pwm for left motor
int SpeedR = 0; //pwm for right motor
int Max_Speed = 60;  //Max pwm that can be given to Motor Driver for Max Speed.
int Max_turn = 65;
int Adj = 0;
int Error = 0;
int L_Error = 0;  //last error, var to store previous error values.
int Goal = 3500;  //3500 is value to keep the robot at centre w.r.t line.
const double Kp = 0.012 * 2;
const double Kd = 0.01 * 10;

void setup() {
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  QTR.setTypeAnalog();
  QTR.setSensorPins((const uint8_t[]){
                      A7, A6, 4, 5, A3, A2, A1, A0 },
                    sensors);
  QTR.setEmitterPin(E_pin);
  for (uint16_t i = 0; i < 400; i++) {
    QTR.calibrate();
  }
}


void PID() {
  Position = QTR.readLineWhite(Sensor_Readings);
  Serial.println(Position);

  Error = Goal - Position;
  //Calculating adjustment to pwm for left & right motors
  Adj = Kp * Error + Kd * (Error - L_Error);
  L_Error = Error;
  SpeedL = Max_Speed - Adj;
  SpeedR = Max_Speed + Adj;

  if (SpeedL > Max_Speed) {
    SpeedL = Max_Speed;
  } else if (SpeedL < 0) {
    SpeedL = 0;
  }
  if (SpeedR > Max_Speed) {
    SpeedR = Max_Speed;
  } else if (SpeedR < 0) {
    SpeedR = 0;
  }

  //Serial.print(SpeedL);
  //Serial.print(" , ");
  //Serial.println(SpeedR);
}

void frd_PWM(int L, int R) {
  analogWrite(IN1, L);
  analogWrite(IN2, 0);
  analogWrite(IN3, R);
  analogWrite(IN4, 0);
}

void bck(int L, int R) {
  analogWrite(IN1, 0);
  analogWrite(IN2, L);
  analogWrite(IN3, 0);
  analogWrite(IN4, R);
}
void sharpR(int L, int R) {
  analogWrite(IN1, L);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, R);
}
void sharpL(int L, int R) {
  analogWrite(IN1, 0);
  analogWrite(IN2, L);
  analogWrite(IN3, R);
  analogWrite(IN4, 0);
}
void stopp() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

//check_side() is used here to get back on line if line is lost completely.

void check_side() {
  if (Position == 0) {
    do {
      sharpL(Max_turn, Max_turn);
      Position = QTR.readLineWhite(Sensor_Readings);
      Serial.println(Position);
      if ((Position > 3200) && (Position < 3800)) {
        break;
      }
    } while (1);
  } else if (Position == 7000) {
    do {
      sharpR(Max_turn, Max_turn);
      Position = QTR.readLineWhite(Sensor_Readings);
      Serial.println(Position);
      if ((Position > 3200) && (Position < 3800)) {
        break;
      }
    } while (1);
  }
}

void loop() {
  PID();
  frd_PWM(SpeedL, SpeedR);
  check_side();
}