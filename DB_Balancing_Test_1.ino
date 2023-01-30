#include "Wire.h"
#include <MPU6050_light.h> 
MPU6050 mpu(Wire);
//PID
#include <PID_v1.h>

int sensorValue =0;
int joyX = 27;

float velocity;
float accum;

float a = 0;

float alpha_dot;
float alpha;

double Pk1 = 6;           // balancing PID values
double Ik1 = 22;
double Dk1 = 0.02; 

//Declare NIDEC Motor pins
#define brake         5  //brake=0, go=1
#define cw            18  //cw=1, ccw=0
#define rpm           19  //PWM=255=stop, PWM=0=max_speed  
/////////////NIDEC Motor//////////////

double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup - motor position

void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  
  pinMode(joyX, INPUT);
  
  byte status = mpu.begin(); //MPU initialization and calibration
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println("MPU begin done!\n");
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  Serial.println("Begin Device initialization:\n");
  nidec_motor_init();
  Serial.println("NIDEC initialized\n");

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-100, 100);
  PID1.SetSampleTime(10);
}
void loop() 
{ sensorValue = analogRead(joyX);
  int mapVal = map(sensorValue, 0, 4096,-255,255);
  //Serial.println(mapVal);
  delay(1);
 
      mpu.update();
      alpha = mpu.getAngleX();
      alpha_dot = mpu.getGyroX();

      
      Input1 = alpha;
      Setpoint1 = 0 - accum;

      accum = accum - (Output1/100);    // observation controller
      accum = (constrain(accum,-1,1));  // constrain the data

                  // debug      
      
      PID1.Compute();                   // compute PID

      velocity = map(Output1,-100,100,-255,255);  
      
 
  Serial.print(alpha);
  Serial.print(" ");
  Serial.print(Output1);
  Serial.print(" ");
  Serial.print(accum);
  Serial.print(" ");
  Serial.println(velocity);
  
  nidec_motor_control(-velocity);
}

void nidec_motor_init()
{ 
  pinMode(brake, OUTPUT);
  pinMode(cw, OUTPUT);
  pinMode(rpm, OUTPUT);
  
  digitalWrite(brake, HIGH);  //go=1
  digitalWrite(cw, LOW);      //direction ccw
  analogWrite(rpm, 255);
}
void nidec_motor_control(int pwm) 
{ 
  if (pwm < 0) 
  { digitalWrite(cw, HIGH);
    pwm = -pwm;} 
  else { digitalWrite(cw, LOW); }
  analogWrite(rpm, 255-pwm);
}
void nidec_motor_brake() 
{ 
  digitalWrite(brake, LOW);  //go=1
  analogWrite(rpm, 255);
}
//////////////////////////////////////
