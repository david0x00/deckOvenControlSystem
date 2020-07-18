#include <SPI.h>
#include "Adafruit_MAX31855.h"

#define MAXDO   3
#define MAXCS   4
#define MAXCLK  5

Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

int relay = 8;

double curr_temp = 0;
double prev_temp = 0;
double ref_temp = 100;

double P, I, D;
double Kp = 1;
double Ki = 0;
double Kd = 10;

double alpha = 0.1;

double error = 0;
double prev_error = 0;
double integral = 0;
double derivative = 0;

double control;

bool print_flag = false;
bool heater_state = false;

void heatOn() {
  if (print_flag){
    Serial.println("heat On");
  }
  if (heater_state == false) {
    integral = 0;
  }
  heater_state = true;
  digitalWrite(relay, HIGH);
}

void heatOff() {
  if (print_flag) {
    Serial.println("heat Off");
  }
  if (heater_state == true) {
    integral = 0;
  }
  heater_state = false;
  digitalWrite(relay, LOW);
}

double getTime() {
  double time;
  time = millis();
  time = time / 1000;
  return time;
}

double getTemperature() {
  double temp;
  temp = thermocouple.readFahrenheit();
  return temp;
}

void printInfo() {
  print_flag = true;
  Serial.print("reference: ");
  Serial.print(ref_temp);
  Serial.print(", temp: ");
  Serial.print(curr_temp);
  Serial.print(", error: ");
  Serial.println(error);
  
  Serial.print("P: ");
  Serial.print(P);
  Serial.print(", I: ");
  Serial.print(I);
  Serial.print(", D: ");
  Serial.print(D);
  Serial.print(", control: ");
  Serial.println(control);
}

void plotTemp() {
  print_flag = false;
  Serial.println(curr_temp);
}

void setup() {
  Serial.begin(9600);
  pinMode(relay, OUTPUT);
  P = 0;
  I = 0;
  D = 0;

  while (!Serial) delay(1);
  delay(500);
  curr_temp = getTemperature();
}

void loop() {
  prev_temp = curr_temp;
  curr_temp = alpha*curr_temp + (1.0-alpha)*getTemperature();
  prev_error = error;

  error = ref_temp - curr_temp;
  integral += error;
  derivative = error - prev_error;

  P = Kp * error;
  I = Ki * integral;
  D = Kd * derivative; 

  control = P + I + D;

  //printInfo();  
  plotTemp();

  if (control > 0) {
    heatOn();
  } else {
    heatOff();
  }

  delay(1000);
}
