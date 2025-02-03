




#include <AFMotor.h>


AF_DCMotor motor1(1, MOTOR12_64KHZ); // create motor #1, 64KHz pwm
AF_DCMotor motor2(2, MOTOR12_64KHZ); // create motor #2, 64KHz pwm

int motor_speed = 0;


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps

  motor_speed = 50;

  forward();
  stop();
  backward();
  stop();
  right();
  stop();
  left();
  stop();
}

void loop() {
  // put your main code here, to run repeatedly:



}

void forward(){
  motor1.setSpeed(motor_speed);   // set the speed to 200/255 pwm signal
  motor2.setSpeed(motor_speed);   // set the speed to 200/255 pwm signal
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  delay(1000);
}
void backward(){
  motor1.setSpeed(motor_speed);   // set the speed to 200/255 pwm signal
  motor2.setSpeed(motor_speed);   // set the speed to 200/255 pwm signal
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  delay(1000);
}
void stop(){
  motor1.run(RELEASE);
  motor2.run(RELEASE);
}
void right(){
  motor1.setSpeed(motor_speed);   // set the speed to 200/255 pwm signal
  motor2.setSpeed(motor_speed);   // set the speed to 200/255 pwm signal
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  delay(1000);
}
void left(){
  motor1.setSpeed(motor_speed);   // set the speed to 200/255 pwm signal
  motor2.setSpeed(motor_speed);   // set the speed to 200/255 pwm signal
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  delay(1000);
}