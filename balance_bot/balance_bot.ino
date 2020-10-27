#include<Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// constants
const int MPU=0x68; 
const int DT = 10;
const double g = 980; // cm /s ^2

// all in meters
const double height = 16.5;  // cm
const double wheel_radius = 3.5; // cm
const double mass = 390; // grams
const double I = (1/3.0)*mass*height*height; // gram * cm ^2
const double height_cg = 7;


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
Adafruit_MPU6050 mpu;
struct gyro_t gyro;
struct motor_t right_motor;
struct motor_t left_motor;
double total_theta = 0;

// for timing
unsigned long previousMillis = 0;

//filter
float Q_angle  =  0.01;
float Q_gyro   =  0.0003;
float R_angle  =  0.01;
float x_bias = 0;
float y_bias = 0;
float XP_00 = 0, XP_01 = 0, XP_10 = 0, XP_11 = 0;
float YP_00 = 0, YP_01 = 0, YP_10 = 0, YP_11 = 0;
float KFangleX = 0.0;
float KFangleY = 0.0;



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

   // Try to initialize mpu
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  Serial.begin(9600); // Starting Serial Terminal
}

void loop() {
  
 
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= DT) {
    previousMillis = currentMillis;
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float angle = RAD_TO_DEG* atan2(a.acceleration.y, a.acceleration.z);
    total_theta = kalmanFilterY(angle, g.gyro.x);

    #if 0
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);

    Serial.print("  A");
    Serial.print(RAD_TO_DEG* angle);

    Serial.print("gx:   ");
    Serial.print(g.gyro.x);

    Serial.print("Total theta: ");
    Serial.print(total_theta);

    Serial.println("");
#endif
    

    
    float value = 5*total_theta;
    set_motor_speed(value, right_motor, left_motor);
#if 1
    Serial.print("Data: ");
    Serial.print(" | DEG_TO_RAD*gyro.gy_y = "); Serial.print(total_theta);
    Serial.print(" | value = "); Serial.print(value);
      Serial.println(" ");

#endif
    
#if 0
    Serial.print("Gyroscope: ");
    Serial.print(" | Y = "); Serial.print(gyro.gy_y);
    Serial.print(" | C = "); Serial.print(translate);
    Serial.print(" | total theta = "); Serial.print(total_theta);
      Serial.println(" ");
#endif
  }
  


}


float kalmanFilterY(float accAngle, float gyroRate)
{
  float  y, S;
  float K_0, K_1;
 
  KFangleY += DT * (gyroRate - y_bias);
 
  YP_00 +=  - DT * (YP_10 + YP_01) + Q_angle * DT;
  YP_01 +=  - DT * YP_11;
  YP_10 +=  - DT * YP_11;
  YP_11 +=  + Q_gyro * DT;
 
  y = accAngle - KFangleY;
  S = YP_00 + R_angle;
  K_0 = YP_00 / S;
  K_1 = YP_10 / S;
 
  KFangleY +=  K_0 * y;
  y_bias  +=  K_1 * y;
  YP_00 -= K_0 * YP_00;
  YP_01 -= K_0 * YP_01;
  YP_10 -= K_1 * YP_00;
  YP_11 -= K_1 * YP_01;
 
  return KFangleY;
}




//velocity between -255 and 255
// 255 = 0.984 second/rev
// 255/2 = 1.617 second/rev
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
