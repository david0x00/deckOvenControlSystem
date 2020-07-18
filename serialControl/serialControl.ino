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
double Kd = 0;
double error = 0;

double control;

bool print_flag = false;


void setup() {
  Serial.begin(9600);
  pinMode(relay, OUTPUT);

  while (!Serial) delay(1);
  delay(500);
}

void heatOn() {
  if (print_flag){
    Serial.println("heat On");
  }
  digitalWrite(relay, HIGH);
}

void heatOff() {
  if (print_flag) {
    Serial.println("heat Off");
  }
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
}

void plotTemp() {
  print_flag = false;
  Serial.println(curr_temp);
}

void loop() {
  prev_temp = curr_temp;
  curr_temp = getTemperature();
  error = ref_temp - curr_temp;

  P = Kp * error;

  control = P;

  //printInfo();  
  plotTemp();

  if (control > 0) {
    heatOn();
  } else {
    heatOff();
  }

  delay(1000);
}
