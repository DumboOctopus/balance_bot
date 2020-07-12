#include<Wire.h>

// constants
const int MPU=0x68; 
const int DELAY = 50;
const double D_CLOCK = DELAY / 1000.0;

// all in centimeters
const double height = 16.5; 
const double wheel_radius = 3.5; 


// structs
struct ultrasonic_t{
  int echoPin;
  int pingPin;
};
struct gyro_t {
  double ac_x,ac_y,ac_z;
  double gy_x, gy_y, gy_z;
};
struct motor_t {
  int enable, in1, in2;
};




// global vars.
struct ultrasonic_t ultra_down;
struct ultrasonic_t ultra_forward;
struct gyro_t gyro;
struct motor_t right_motor;
struct motor_t left_motor;
double total_theta = 0;


void setup() {
  ultra_down.echoPin = 11;
  ultra_down.pingPin = 12;
  
  ultra_forward.echoPin = 3;
  ultra_forward.pingPin = 4;

  // gyro 
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);

  //motor right
  right_motor.enable = 5;
  right_motor.in1 = 8;
  right_motor.in2 = 7;

  pinMode(right_motor.enable, OUTPUT);
  pinMode(right_motor.in1, OUTPUT);
  pinMode(right_motor.in2, OUTPUT);

  //motor left
  left_motor.enable = 6;
  left_motor.in1 = 10;
  left_motor.in2 = 9;

  pinMode(left_motor.enable, OUTPUT);
  pinMode(left_motor.in1, OUTPUT);
  pinMode(left_motor.in2, OUTPUT);
  
  Serial.begin(9600); // Starting Serial Terminal
}

void loop() {
  long duration, inches, cm;
  cm = read_ultrasonic(ultra_forward);
  read_gyro(gyro);
  
  double dxdt = height * sin(DEG_TO_RAD *total_theta);
  double dthdt = dxdt/wheel_radius;

  set_motor_speed(dthdt*255*10, right_motor, left_motor);
  
 // Serial.print("Gyroscope: ");
 // Serial.print(" | T = "); Serial.print(second_to_rad *total_theta);
 // Serial.print(" | Y = "); Serial.print(gyro.gy_y);
 // Serial.print(" | total theta = "); Serial.print(total_theta);
 // Serial.print(" | dx/dt = "); Serial.print(dxdt);

 // Serial.println(" ");

   
  delay(DELAY);
}

void read_gyro(struct gyro_t &g){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);  
  g.ac_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  g.ac_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  g.ac_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)

  g.gy_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  g.gy_y = (int16_t)(Wire.read()<<8 | Wire.read()); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  
  g.gy_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  g.gy_x *= 1/1800.0; // minutes
  g.gy_y *= 1/1800.0;
  g.gy_z *= 1/1800.0;
  if(-0.1 <= g.gy_y && g.gy_y <= 0.1)
    g.gy_y = 0; 
  
  
  total_theta += gyro.gy_y;


  Serial.print("Gyroscope: ");
  Serial.print(" | X = "); Serial.print(gyro.gy_x);
  Serial.print(" | Y = "); Serial.print(gyro.gy_y);
  Serial.print(" | Z = "); Serial.print(gyro.gy_z);
  Serial.print(" | total theta = "); Serial.print(total_theta);
  Serial.println(" ");
  
  
}



int read_ultrasonic(struct ultrasonic_t ultrasonic){
  long duration, inches, cm;
   pinMode(ultrasonic.pingPin, OUTPUT);
   digitalWrite(ultrasonic.pingPin, LOW);
   delayMicroseconds(2);
   digitalWrite(ultrasonic.pingPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(ultrasonic.pingPin, LOW);
   pinMode(ultrasonic.echoPin, INPUT);
   duration = pulseIn(ultrasonic.echoPin, HIGH);
   
   cm = microsecondsToCentimeters(duration);
   
   return cm;
}

//velocity between -255 and 255
void set_motor_speed(int velocity, motor_t a, motor_t b){
  if(velocity == 0){
    digitalWrite(a.in1, LOW);
    digitalWrite(a.in2, LOW);
    analogWrite(a.enable, 0);
    
    digitalWrite(b.in1, LOW);
    digitalWrite(b.in2, LOW);
    analogWrite(b.enable, 0);
  } else if(velocity > 0){
    digitalWrite(a.in1, LOW);
    digitalWrite(a.in2, HIGH);
    analogWrite(a.enable, min(velocity, 255));
    
    digitalWrite(b.in1, LOW);
    digitalWrite(b.in2, HIGH);
    analogWrite(b.enable, min(velocity, 255));
  } else {
    digitalWrite(a.in1, HIGH);
    digitalWrite(a.in2, LOW);
    analogWrite(a.enable, min(-velocity, 255));
    
    digitalWrite(b.in1, HIGH);
    digitalWrite(b.in2, LOW);
    analogWrite(b.enable, min(-velocity, 255));
  }

}

long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}
