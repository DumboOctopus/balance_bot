#include<Wire.h>

// constants
const int MPU=0x68; 
const int DELAY = 10;
const double g = 9.8;

// all in meters
const double height = 16.5/100.0; 
const double wheel_radius = 3.5/100.0; 
const double mass = 0.390; // killo grams


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

// for timing
unsigned long previousMillis = 0;
unsigned long gyZeroClock = 0;
bool isZero = true;


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

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= DELAY) {
    previousMillis = currentMillis;
    
    
    read_gyro(gyro);
    bool newIsZero = -0.001 <= gyro.gy_y && gyro.gy_y <= 0.001;
    if(!isZero && newIsZero){
      gyZeroClock = currentMillis;
    } else if(isZero && !newIsZero){
      unsigned long total = currentMillis - gyZeroClock;
      total_theta -= min(total/500.0, 1) * total_theta;
    }
    isZero = newIsZero;
    

    // formula = h/r cos(theta) * dtheta/dt
    // dtheta/dt = gyro + acceleration calculated from theta
    double I = (1/3.0)*mass*height*height;
    double alpha = height/1.5 * g * sin(DEG_TO_RAD*total_theta)/I;
    
    double dthetadt = DEG_TO_RAD * gyro.gy_y +  DELAY/1000.0*alpha;
    double value = height/wheel_radius * cos(DEG_TO_RAD * total_theta) * dthetadt;
    double translate = 255* value;
    set_motor_speed(translate, right_motor, left_motor);

    //set_motor_speed(255, right_motor, left_motor);  
    

    //Serial.print("Gyroscope: ");
    //Serial.print(" | Y = "); Serial.print(gyro.gy_y);
    //Serial.print(" | C = "); Serial.print(translate);
    //Serial.print(" | total theta = "); Serial.print(total_theta);
    //  Serial.println(" ");
  }
  
  
  

  
  
 // Serial.print("Gyroscope: ");
 // Serial.print(" | T = "); Serial.print(second_to_rad *total_theta);
 // Serial.print(" | Y = "); Serial.print(gyro.gy_y);
 // Serial.print(" | total theta = "); Serial.print(total_theta);
 // Serial.print(" | dx/dt = "); Serial.print(dxdt);

 // Serial.println(" ");

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


double lerp(double input, double end_output) {
   return 255 * input / end_output;
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

long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}
