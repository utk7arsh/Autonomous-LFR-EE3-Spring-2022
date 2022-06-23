#include <ECE3.h> // Used for encoder functionality
uint16_t sensorValues[8];
float errorValues[8];
float minimum[8];
float maximum[8];
float sensorNew[8];
float pastTenError[10]; 
const int left_nslp_pin = 31;
const int left_dir_pin  = 29;
const int left_pwm_pin  = 40;

const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;

float error = 0;
float k_p = 0.01;
float k_d = 0.04; 
float base_motor_right = 70;
float base_motor_left = 70;
int count = 0;  
int sum = 0; 
float difference = 0;
float pastSum = 0; 
int numOfTurns = 0; 
float pastError = 0;

void setup() {
  // put your setup code here, to run once:
  ECE3_Init();
  errorValues[0] = -8;
  errorValues[1] = -4;
  errorValues[2] = -2;
  errorValues[3] = -1;
  errorValues[4] = 1;
  errorValues[5] = 2;
  errorValues[6] = 4;
  errorValues[7] = 8;
 
   minimum[0] = 598;
   minimum[1] = 551;
   minimum[2] = 643;
   minimum[3] = 644;
   minimum[4] = 551;
   minimum[5]= 690;
   minimum[6] = 690;
   minimum[7]= 715;
  
   maximum[0] = 850;
   maximum[1] = 754;
   maximum[2] = 1186;
   maximum[3] = 1066.5;
   maximum[4] = 1073.6;
   maximum[5] = 1030;
   maximum[6] = 1092;
   maximum[7] = 1781;

   for(int i = 0; i < 10; i++)
    pastTenError[i] = 0;

  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);
  pinMode(41, OUTPUT);
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);
  delay(2000);
}

void turnAround(){
  int goalEncoderCount_right = getEncoderCount_right()+160;
  int goalEncoderCount_left = getEncoderCount_left()+160;
  analogWrite(right_pwm_pin,255);
  analogWrite(left_pwm_pin, 255);
  digitalWrite(right_dir_pin, LOW);
  digitalWrite(left_dir_pin, HIGH);
  while((goalEncoderCount_right >= getEncoderCount_right()) || (goalEncoderCount_left >= getEncoderCount_left()))
  {
  }
  digitalWrite(left_dir_pin, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  ECE3_read_IR(sensorValues);
  error = 0;
  sum = 0; 

  for(int i = 0; i < 8; i++)
  {
    sensorNew[i] = sensorValues[i];
    sensorNew[i] -= minimum[i];
    sensorNew[i] = sensorNew[i]/maximum[i]*1000;
    error += errorValues[i]*sensorNew[i]; 
  }
  
  for(int i = 0; i < 8; i++)
  {
    sum += sensorNew[i];
  }

    difference = error - pastError; 

 if((getEncoderCount_right() + getEncoderCount_left())/2 > 1700 && (getEncoderCount_right() + getEncoderCount_left())/2 < 3150)
 {
    k_p = 0.0073;
    k_d = 0.029; 
    base_motor_right = 170;
    base_motor_left = 170;
  }
  else if((getEncoderCount_right() + getEncoderCount_left())/2 > 3150 && (getEncoderCount_right() + getEncoderCount_left())/2 < 9500)
  {
    k_p = 0.019;
    k_d = 0.17; 
    base_motor_right = 100;
    base_motor_left = 100;
  }
  else if((getEncoderCount_right() + getEncoderCount_left())/2 > 9500 && (getEncoderCount_right() + getEncoderCount_left())/2 < 10700)
  {
    k_p = 0.0073;
    k_d = 0.029; 
    base_motor_right = 170;
    base_motor_left = 170;
  }
  else if((getEncoderCount_right() + getEncoderCount_left())/2 > 10000)
  {
    k_p = 0.01;
    k_d = 0.04; 
    base_motor_right = 70;
    base_motor_left = 70;
  }

 
  if(((sum + pastSum)/2) > 8500)
  {
    digitalWrite(41, HIGH);
    numOfTurns++;
    if(numOfTurns >= 2)
    {
      base_motor_left = 0;
      base_motor_right = 0;
      k_p = 0; 
      k_d = 0; 
    }
    else
     turnAround();
    digitalWrite(41, LOW);
  }
  else
  { 
    analogWrite(left_pwm_pin, base_motor_left - error*k_p - k_d*difference);
    analogWrite(right_pwm_pin, base_motor_right + error*k_p + k_d*difference);
  }
  pastSum = sum; 
  pastError = error; 
}
