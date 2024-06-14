#include <Servo.h>

Servo servohori; // Horizontal servo (BOTTOM SERVO)
Servo servoverti; // Vertical servo (TOP SERVO)

int servoh = 90; // Initial position of the horizontal servo
int servohLimitHigh = 180; // Maximum range of the servo is 180 degrees
int servohLimitLow = 0; // Minimum range of the servo is 0 degrees

int servov = 90; // Initial position of the vertical servo
int servovLimitHigh = 180; // Maximum range of the servo is 180 degrees
int servovLimitLow = 0; // Minimum range of the servo is 0 degrees

int ldrtopr = 3; // Top right LDR on pin A1
int ldrtopl = 0; // Top left LDR on pin A2
int ldrbotr = 2; // Bottom right LDR on pin A0
int ldrbotl = 1; // Bottom left LDR on pin A3

// PID variables for horizontal control
float KpH = 0.1; // Proportional gain
float KiH = 0.03; // Integral gain
float KdH = 0; // Derivative gain
float TsH = 0.1; // Sample time (s)

float errorH_prev = 0, integralH = 0;

// PID variables for vertical control
float KpV = 0.1; // Proportional gain
float KiV = 0.03; // Integral gain
float KdV = 0; // Derivative gain
float TsV = 0.1; // Sample time (s)

float errorV_prev = 0, integralV = 0;

// Threshold for light detection
int lightThreshold = 50;

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud rate
  servohori.attach(10); // Horizontal servo connected to Arduino pin 10
  servohori.write(servoh);
  
  servoverti.attach(9); // Vertical servo connected to Arduino pin 9
  servoverti.write(servov);
  delay(500); // Delay to stabilize servos
}

void loop() {
  servoh = servohori.read();
  servov = servoverti.read();

  int topl = analogRead(ldrtopl); // Read analog values from top left LDR
  int topr = analogRead(ldrtopr); // Read analog values from top right LDR
  int botl = analogRead(ldrbotl); // Read analog values from bottom left LDR
  int botr = analogRead(ldrbotr); // Read analog values from bottom right LDR

  int avgtop = (topl + topr) / 2; // Average of top LDRs
  int avgbot = (botl + botr) / 2; // Average of bottom LDRs
  int avgleft = (topl + botl) / 2; // Average of left LDRs
  int avgright = (topr + botr) / 2; // Average of right LDRs

  // Calculate errors
  float errorH = avgleft - avgright;
  float errorV = avgtop - avgbot;

  // PID for horizontal servo
  integralH += errorH * TsH;
  float derivativeH = (errorH - errorH_prev) / TsH;
  float outputH = KpH * errorH + KiH * integralH + KdH * derivativeH;
  servoh = constrain(servoh + outputH, servohLimitLow, servohLimitHigh);
  servohori.write(servoh);
  errorH_prev = errorH;

  // PID for vertical servo
  integralV += errorV * TsV;
  float derivativeV = (errorV - errorV_prev) / TsV;
  float outputV = KpV * errorV + KiV * integralV + KdV * derivativeV;
  servov = constrain(servov + outputV, servovLimitLow, servovLimitHigh);
  servoverti.write(servov);
  errorV_prev = errorV;

  // Send LDR values to Serial Plotter
  Serial.print(topl);
  Serial.print(",");
  Serial.print(topr);
  Serial.print(",");
  Serial.print(botl);
  Serial.print(",");
  Serial.println(botr);
  Serial.print(outputV);
  Serial.print(",");
  Serial.println(outputH);
  Serial.print(",");
  delay(100); // Loop delay in milliseconds (equal to Ts)
}
