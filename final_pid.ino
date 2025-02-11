int motorSpeed;
#include "CytronMotorDriver.h"
CytronMD motor(PWM_DIR, 10, 12);
#define Encoder_A 3
#define Encoder_B 4
float ppr = 600;
volatile int Encoderpos = 0;
float degreesperpulse;

#define ENCA 2
#define ENCB 9
float PPR = 578;
float dia = 0.013;
float distancePerPulse;
volatile int posi = 0;
float position_m;

float TRACK_LENGTH = 1.0;
float MIDDLE_POINT = 0.0;

// float Kp_angle = 7.3825;
// float Ki_angle = 0.000001;
// float Kd_angle = 0.0003665;

float Kp_angle = 7.5125;
float Ki_angle = 0.000000;
float Kd_angle = 0.0042635;

float Kp_pos = 0;//12.0;
float Ki_pos = 0.0;
float Kd_pos = 0;//0.000005;

float angle, angle_error, last_angle_error, angle_integral, angle_derivative;
float angle_output;

float position, pos_error, last_pos_error, pos_integral, pos_derivative;
float pos_output;

#define MAX_SPEED 1
#define MIN_SPEED 0

unsigned long lastTime;
float timeChange;

void setup() {
  Serial.begin(115200);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  pinMode(Encoder_A, INPUT_PULLUP);
  pinMode(Encoder_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), updateEnc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_A), updateEncoder, CHANGE);
  float circum = dia * 3.14159;
  distancePerPulse = circum / PPR;
  degreesperpulse= 180.0/600.0;

  angle_error = 0;
  last_angle_error = 0;
  angle_integral = 0;
  angle_derivative = 0;

  pos_error = 0;
  last_pos_error = 0;
  pos_integral = 0;
  pos_derivative = 0;

  lastTime = millis();
}

void updateEnc() {
  if (digitalRead(ENCA) == digitalRead(ENCB)) {
    posi--;
  } else {
    posi++;
  }
}
void updateEncoder() {
  if (digitalRead(Encoder_A) == digitalRead(Encoder_B)) {
    Encoderpos --;
  } else {
    Encoderpos ++;
  }
}

float readPosition() {
  float position_m = posi * distancePerPulse;
  return position_m;
}

unsigned int readAngle() {
  float angle = Encoderpos * degreesperpulse;
  // Serial.println(-angle);
  if(angle>360)
  {
    return 0;
  }
  return -angle;
}

float calculateAnglePID(float angle) {
 

  angle_error = 180 - angle;

  if(abs(angle_error)>30)
  {
    return 0;
  }

  angle_integral += angle_error ;
  angle_integral = constrain(angle_integral, -20, 20);

  angle_derivative = (angle_error - last_angle_error);

  angle_output = (Kp_angle * angle_error) + (Ki_angle * angle_integral) + (Kd_angle * angle_derivative);

last_angle_error = angle_error;

  return angle_output;
}

float calculatePositionPID(float position) {
  //timeChange = (millis() - lastTime) / 1000.0;
  //lastTime = millis();
  Serial.println(position);
  pos_error = (0 - position)*100;
  Serial.println(pos_error);
  if(abs(pos_error)>42)
  {
    return 0;
  }

  pos_integral += pos_error ;
  pos_integral = constrain(pos_integral, -20, 20);

  pos_derivative = (pos_error - last_pos_error) ;

  pos_output = (Kp_pos * pos_error) + (Ki_pos * pos_integral) + (Kd_pos * pos_derivative);

  last_pos_error = pos_error;

  return pos_output;
}

void controlMotor(float velocity) {
  motor.setSpeed(velocity);
}

void loop() {
  position = readPosition();
 unsigned int rawAngle = readAngle(); // Read raw angle
  angle_output = calculateAnglePID(rawAngle);
  pos_output = calculatePositionPID(position);
  int combined_output = angle_output + pos_output;

  if(abs(combined_output)<250)
  {
  controlMotor(combined_output);
  Serial.println(combined_output);
  }
  

  Serial.print("Max:");
  Serial.print(360);
  Serial.print(",Min:");
  Serial.print(0);
  Serial.print(",Mid:");
  Serial.print(180);
  Serial.print(",Angle:");
  Serial.println(rawAngle);
  Serial.print(",POs output:");
  Serial.println(pos_output);
  // delay(10);
}