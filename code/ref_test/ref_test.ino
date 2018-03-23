#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

// motor drving pins
#define LED 13
#define PWM_E1 6
#define PWM_E2 5
#define DIR_M1 7
#define DIR_M2 4

// Hall senser pins
#define HA_left 8
#define HB_left 9
#define HA_right 12
#define HB_right 11

// PID gains 
#define Kp  100
#define Kd  2
#define Ki  
#define incTime  0.005
#define targetAngle 0.002

// LQR gains
#define Ky 20
#define Kdy 30
#define Kth -204
#define Kdth -62
//#define CPR 980

MPU6050 mpu;

int16_t accY, accZ, gyroX;
volatile int gyroRate;
volatile long motorPWM;
volatile float accAngle, gyroAngle, currAngle, prevAngle=0, err, prevErr=0, errSum=0;
volatile byte count=0;
volatile float y, dy, prevy=0; 
volatile int LA_currState, RA_currState, LA_prevState, RA_prevState, encode=0;
volatile float CPR=490;

void init_Control() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 2.5ms
  OCR1A = 9999;   
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  
  // (4999+1)*8/16M = 2.5 ms, 400 Hz
  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void setup() {
  // set the motor direction and PWM pins to output mode
  pinMode(PWM_E1, OUTPUT);
  pinMode(PWM_E2, OUTPUT);
  pinMode(DIR_M1, OUTPUT);
  pinMode(DIR_M2, OUTPUT);
  
  // set motor Hall sensor pins
  pinMode(HA_left, INPUT);
  pinMode(HB_left, INPUT);
  pinMode(HA_right, INPUT);
  pinMode(HB_right, INPUT);
  
  // set the status LED to output mode 
  pinMode(LED, OUTPUT);
  // initialize the MPU6050 and set offset values
  mpu.initialize();
  mpu.setYAccelOffset(-2500);
  mpu.setZAccelOffset(1389);
  mpu.setXGyroOffset(44);
  // initialize PID sampling loop
  init_Control();
  
  Serial.begin(9600);
  
  // record inital Hall sensors reading
  LA_prevState = digitalRead(HA_left);
  RA_prevState = digitalRead(HA_right);
}

void setMotors(int leftPWM, int rightPWM) {
  if(leftPWM >= 0) {
    analogWrite(PWM_E1, leftPWM);
    digitalWrite(DIR_M1, HIGH);
  }
  else {
    analogWrite(PWM_E1, -1*leftPWM);
    digitalWrite(DIR_M1, LOW);
  }
  if(rightPWM >= 0) {
    analogWrite(PWM_E2, rightPWM);
    digitalWrite(DIR_M2, HIGH);
  }
  else {
    analogWrite(PWM_E2, -1*rightPWM);
    digitalWrite(DIR_M2, LOW);
  }
}

void loop() {
  // read acceleration and gyroscope values
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();  
  gyroX = mpu.getRotationX();
  
  setMotors(motorPWM, motorPWM);
  //Serial.println(motorPWM);
}

// The ISR will be called every 2.5 milliseconds
ISR(TIMER1_COMPA_vect)
{ 
  // set motor power after constraining it
  // calculate the angle of inclination
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*incTime;  
  
  // with low and high pass filter
  currAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
  
  // claculate angle error 
  err = currAngle - targetAngle;
  errSum += err;  
  errSum = constrain(errSum, -600, 600);
  
  // calculate PWM output with PID 
  //motorPWM = Kp*err + Ki*errSum*incTime + Kd*(currAngle-prevAngle)/incTime;
  
  // read encoder value
  LA_currState = digitalRead(HA_left);
  RA_currState = digitalRead(HA_right);
  
  // implement encoder counts
  if (LA_currState != LA_prevState) {
    if (digitalRead(HB_left) != LA_currState) {
      encode ++;
    }
    else {
      encode --;
    }
  }
  LA_prevState = LA_currState;
  
  if (encode > CPR*3 || encode < CPR*3) {
    encode = 0;
  }
  
  //calculate y using encoder
  y = (float)(encode/CPR)*0.08*3.1415926;
  
  // calculate PWM output with LQR
  //y = accY*0.00000854492;
  dy = (y-prevy)/incTime;
  motorPWM = 10*(-Ky*y - Kdy*dy - Kth*err*DEG_TO_RAD - Kdth*(currAngle-prevAngle)*DEG_TO_RAD/incTime);
  
  motorPWM = constrain(motorPWM, -255, 255);
  prevAngle = currAngle;
  prevy = y;
  
  // toggle the led on pin13 every second
  count++;
  if(count == 200)  {
    count = 0;
    digitalWrite(LED, !digitalRead(LED));
  }
}
