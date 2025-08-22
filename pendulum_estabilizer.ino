#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.1415926
#endif

const int pinMotorENA    = 8;
const int pinMotorIN1    = 7;
const int pinMotorIN2    = 6;
const int pinLimitLeft   = 2;
const int pinLimitRight  = 3;
const int pinEncoderA    = 18;


const int potPinKp = A0;
const int potPinKi = A1;
const int potPinKd = A2;

MPU6050 mpu;
int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
long bias_ax = 0, bias_ay = 0, bias_az = 0;
long bias_gx = 0, bias_gy = 0, bias_gz = 0;

const float alpha = 0.96f;
unsigned long lastMicros;

volatile long pulseCount = 0;
long encoderPos = 0;
const int pulsesPerRev     = 437;
const float beltPitch      = 0.002;
const int pulleyTeeth      = 20;
const float wheelCircumf   = pulleyTeeth * beltPitch;
const float linearPerPulse = wheelCircumf / pulsesPerRev;

int inverte = 0;
unsigned long lastTimeEnc = 0;

volatile bool limitLeftHit  = false;
volatile bool limitRightHit = false;
volatile unsigned long limitLeftMillis  = 0;
volatile unsigned long limitRightMillis = 0;

static bool returningHome = false;
static bool stopAll       = false;

static bool waitAfterLimit      = false;
static unsigned long waitStartTime = 0;
static bool delayAfterCenter    = false;
static unsigned long centerWaitStartTime = 0;

const float Ts_sec = 0.127f;
const unsigned long Ts_ms = 127;
unsigned long lastControlMs = 0;

const float a1 = 1.1;
const float a2 = -0.1;
float Kp = -3.4600;
float Ki = -0.8420;
float Kd = -3.5543;

float uk1 = 0, uk2 = 0;
float ek1 = 0, ek2 = 0;
float rfk1 = 0, rk1 = 0;

float y=0;
float u_control = 0;
float e = 0;
float u = 0;
float r = 0.0;
int pwmControl = 0;
int dirControl = 0;

void calibrateSensors();
void encoderISR();
void ISR_limitLeft();
void ISR_limitRight();
void stopMotor();
void applyPWM(int dir, int pwm);
void readGainsFromPotentiometers();


void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(pinMotorENA, OUTPUT);
  pinMode(pinMotorIN1, OUTPUT);
  pinMode(pinMotorIN2, OUTPUT);
  pinMode(pinLimitLeft,  INPUT_PULLUP);
  pinMode(pinLimitRight, INPUT_PULLUP);
  pinMode(pinEncoderA,   INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinEncoderA), encoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(pinLimitLeft),  ISR_limitLeft,  FALLING);
  attachInterrupt(digitalPinToInterrupt(pinLimitRight), ISR_limitRight, FALLING);

  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println(F("Erro: MPU6050 não encontrado"));
    while (1);
  }
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  calibrateSensors();

  lastTimeEnc = millis();
  lastMicros  = micros();
  lastControlMs = millis();
  Serial.println(F("Planta + Controlador digital carregados."));
}

void loop() {

  readGainsFromPotentiometers();

  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMicros) / 1e6f;
  lastMicros = currentMicros;

  mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

  float ax = ax_raw / 16384.0f;
  float ay = ay_raw / 16384.0f;
  float az = az_raw / 16384.0f;
  float gx_dps = gx_raw / 131.0f;
  float gy_dps = gy_raw / 131.0f;
  float gz_dps = gz_raw / 131.0f;

  float angleAccel = atan2(ay, ax);

  if (angleAccel > M_PI_2)   angleAccel = M_PI - angleAccel;
  if (angleAccel < -M_PI_2)  angleAccel = -M_PI - angleAccel;

  if (angleAccel > 85.0 * M_PI / 180.0 && angleAccel < 95.0 * M_PI / 180.0) angleAccel = M_PI_2;
  if (angleAccel < -85.0 * M_PI / 180.0 && angleAccel > -95.0 * M_PI / 180.0) angleAccel = -M_PI_2;

  static float angleFilt = 0.0;
  float gzRate = gz_dps * (M_PI / 180.0f); // agora em rad/s
  angleFilt = alpha * (angleFilt + gzRate * dt) + (1.0f - alpha) * angleAccel;

  if (angleFilt > M_PI_2)  angleFilt = M_PI_2;
  if (angleFilt < -M_PI_2) angleFilt = -M_PI_2;

  bool sensorLeft  = digitalRead(pinLimitLeft);
  bool sensorRight = digitalRead(pinLimitRight);

  static bool sensorLeftLatched  = false;
  static bool sensorRightLatched = false;

  if (sensorLeft && !sensorLeftLatched && !returningHome && !delayAfterCenter) {
      sensorLeftLatched = true;
      inverte = !inverte;
      returningHome = true;
      Serial.println(F(">> Fim de curso ESQUERDA!"));
  }
  if (!sensorLeft)  sensorLeftLatched  = false;

  if (sensorRight && !sensorRightLatched && !returningHome && !delayAfterCenter) {
      sensorRightLatched = true;
      inverte = !inverte;
      returningHome = true;
      Serial.println(F(">> Fim de curso DIREITA!"));
  }
  if (!sensorRight) sensorRightLatched = false;

  if (!returningHome && !waitAfterLimit && !delayAfterCenter && !stopAll) {
      unsigned long now_ms = millis();
      if (now_ms - lastControlMs >= Ts_ms) {
          lastControlMs = now_ms;
          u=0;

         y = angleFilt;
          r = 0.0f;

        if(y<0.06 && y > -0.06){
          y=0;
        }

          e = r - y;

          u = (a1*uk1 + a2*uk2 + Kd*e + (Kp-2*Kd)*ek1 + (Ki+Kd-Kp)*ek2);


        if(y<0.06 && y > -0.06){
          u=0;
        }

          u_control = u;


          ek2 = ek1;
          ek1 = e;

          uk2 = uk1;
          uk1 = u;


          dirControl = (u_control > 0) ? +1 : (u_control < 0 ? -1:0);
          pwmControl = (int)(fabs(u_control)/12.0f*255.0f);
          pwmControl = constrain(pwmControl, 50, 255);
      }
  }

  if (waitAfterLimit) {
      stopMotor();
      if (millis() - waitStartTime >= 4000UL) {
          waitAfterLimit = false;
          returningHome  = true;
      }
      delay(5);
      return;
  }

  if (delayAfterCenter) {
      stopMotor();
      if (millis() - centerWaitStartTime >= 5000UL) {
          delayAfterCenter = false;
          stopAll = false;
      }
      delay(5);
      return;
  }

  int motorDirection = 0;
  int pwmToApply     = 255;

  if (returningHome) {
      if      (encoderPos > 0)  motorDirection = -1;
      else if (encoderPos < 0)  motorDirection = +1;
      else {
          returningHome     = false;
          stopAll           = true;
          delayAfterCenter  = true;
          centerWaitStartTime = millis();
      }
  }
  else if (!stopAll) {
      motorDirection = dirControl;
      pwmToApply     = pwmControl;
  }

  applyPWM(motorDirection, pwmToApply);

  unsigned long now = millis();
  float dt_enc = (now - lastTimeEnc)/1000.0f;
  lastTimeEnc = now;
  if (dt_enc <= 0) dt_enc = 0.001f;

  long pulses = pulseCount; pulseCount = 0;
  encoderPos += pulses * ((motorDirection==0)? 0 : motorDirection);
  float positionLinear = fabs(encoderPos * linearPerPulse);
  float velocityLinear = (pulses * linearPerPulse) / dt_enc;
  float motorRPM       = (pulses * 60.0f) / (pulsesPerRev * dt_enc);

  Serial.print(F("  uk1: "));       Serial.print(uk1,2);
  Serial.print(F("  uk2: "));       Serial.print(uk2,2);
  Serial.print(F("  ek1: "));       Serial.print(ek1,2);
  Serial.print(F("  ek2: "));       Serial.print(ek2,2);
  Serial.print(F("  Ang(rad): ")); Serial.print(y,4);
  Serial.print(F(" e: ")); Serial.print(e, 4);
  Serial.print(F("  u: "));     Serial.println(u,4);
}


void applyPWM(int dir, int pwm) {
  pwm = constrain(pwm, 0, 255);
  if (dir > 0) {
      digitalWrite(pinMotorIN1, HIGH);
      digitalWrite(pinMotorIN2, LOW);
      analogWrite(pinMotorENA, pwm);
  }
  else if (dir < 0) {
      digitalWrite(pinMotorIN1, LOW);
      digitalWrite(pinMotorIN2, HIGH);
      analogWrite(pinMotorENA, pwm);
  }
  else {
      stopMotor();
  }
}

void stopMotor() {
  digitalWrite(pinMotorIN1, LOW);
  digitalWrite(pinMotorIN2, LOW);
  analogWrite(pinMotorENA, 0);
}

void calibrateSensors() {
  const int discardSamples = 100;
  const int calibSamples   = 200;
  long sum_ax=0, sum_ay=0, sum_az=0, sum_gx=0, sum_gy=0, sum_gz=0;

  stopMotor();
  for (int i=0; i<discardSamples; i++) {
      mpu.getMotion6(&ax_raw,&ay_raw,&az_raw,&gx_raw,&gy_raw,&gz_raw);
      delay(5);
  }
  for (int i=0; i<calibSamples; i++) {
      mpu.getMotion6(&ax_raw,&ay_raw,&az_raw,&gx_raw,&gy_raw,&gz_raw);
      sum_ax+=ax_raw; sum_ay+=ay_raw; sum_az+=az_raw;
      sum_gx+=gx_raw; sum_gy+=gy_raw; sum_gz+=gz_raw;
      delay(10);
  }
  bias_ax = sum_ax / calibSamples;
  bias_ay = sum_ay / calibSamples;
  bias_az = sum_az / calibSamples - 16384;
  bias_gx = sum_gx / calibSamples;
  bias_gy = sum_gy / calibSamples;
  bias_gz = sum_gz / calibSamples;
}

void encoderISR() { pulseCount++; }
void ISR_limitLeft()  { limitLeftHit  = true; limitLeftMillis  = millis(); }
void ISR_limitRight() { limitRightHit = true; limitRightMillis = millis(); }

void readGainsFromPotentiometers() {
  int rawKp = analogRead(potPinKp);
  int rawKi = analogRead(potPinKi);
  int rawKd = analogRead(potPinKd);

  const float Kp_max = -3.4600;
  const float Ki_max = -0.8420;
  const float Kd_max = -3.5543;

  Kp = Kp_max * (rawKp/1023.0);
  Ki = Ki_max * (rawKi/1023.0);
  Kd = Kd_max * (rawKd/1023.0);

  Serial.print("Kp: "); Serial.print(Kp, 4);
  Serial.print(" | Ki: "); Serial.print(Ki, 4);
  Serial.print(" | Kd: "); Serial.println(Kd, 4);
}