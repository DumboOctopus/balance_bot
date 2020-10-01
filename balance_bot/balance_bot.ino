#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// constants
const int MPU=0x68; 
const int DELAY = 10;
int count=0;
const double gravity = 980; // cm /s ^2

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

struct motor_t {
  int enable, in1, in2;
};




// global vars.
struct ultrasonic_t ultra_down;
struct ultrasonic_t ultra_forward;
Adafruit_MPU6050 mpu;
struct motor_t right_motor;
struct motor_t left_motor;
double total_theta = 0;

double pitch;
double lastError = 0;
double Kp= 2.5, Ki=30/1000.0, Kd=0;

float filteredAccX=0;
float filteredAccY=-9.8;
float filteredAccZ=0;

// for timing
unsigned long previousMillis = 0;



void setup() {
  Serial.begin(9600); // Starting Serial Terminal

  ultra_down.echoPin = 11;
  ultra_down.pingPin = 12;
  
  ultra_forward.echoPin = 3;
  ultra_forward.pingPin = 4;


  // gyro
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.print("hi");

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
  
}

void loop() {
  
//  long duration, inches, cm;
//  cm = read_ultrasonic(ultra_forward);

  unsigned long currentMillis = millis();
  unsigned long elapsedTime = currentMillis - previousMillis;


  if (elapsedTime >= DELAY) {
    float dt = elapsedTime/1000.0;
   /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
  
    /* Print out the values */
#if 0
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);

    Serial.println(" m/s^2");
#endif
    float accXRaw = a.acceleration.x;
    float accYRaw = a.acceleration.y;
    float accZRaw = a.acceleration.z;

    float alpha = 0.5;
    filteredAccX = alpha*accXRaw + (1-alpha)*filteredAccX;
    filteredAccY = alpha*accYRaw + (1-alpha)*filteredAccY;

    filteredAccZ = alpha*accZRaw + (1-alpha)*filteredAccZ;

#if 0
    Serial.print("Acceleration X: ");
    Serial.print(filteredAccX);
    Serial.print(", Y: ");
    Serial.print(filteredAccY);
    Serial.print(", Z: ");
    Serial.print(filteredAccZ);

    Serial.println(" m/s^2");
#endif

    float roll;
    if(-0.1 < filteredAccZ  && filteredAccZ < 0.1)
      roll = 0;
    else
      roll = atan2(-filteredAccY,filteredAccZ)*RAD_TO_DEG-90;//90+ atan2(accY, sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;

    float error = f_error(roll - RAD_TO_DEG*g.gyro.x*dt);
    double cumError = error * elapsedTime;
    double rateError = (error - lastError)/dt;
    int output = (int)(Kp * error + Ki * cumError + Kd * rateError);
    lastError = error;

    
    set_motor_speed(output, right_motor, left_motor);

#if 1
    Serial.print("Rotation X: ");
    Serial.print(pitch);
    Serial.print(", Output: ");
    Serial.print(output);
    Serial.print(" cumError: ");
    Serial.print(cumError);
    Serial.println(" x");
    
#endif 
//    Serial.print("Rotation X: ");
//    Serial.print(g.gyro.x);
//    Serial.print(", Y: ");
//    Serial.print(g.gyro.y);
//    Serial.print(", Z: ");
//    Serial.print(g.gyro.z);
//    Serial.println(" rad/s");
//    Serial.println(millis() - currentMillis);
     previousMillis = currentMillis;
  }

}


float f_error(float roll){
    return -roll;
    /*
    roll /= 5;
    if(roll < 0) {
       return roll*roll;
    } else {
      return -roll * roll;
    }*/
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
  velocity = constrain(velocity, -100, 100);
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
    analogWrite(a.enable, map(velocity, 0, 100, 70, 255));
    
    digitalWrite(b.in1, LOW);
    digitalWrite(b.in2, HIGH);
    analogWrite(b.enable, map(velocity, 0, 100, 70, 255));
  } else {
    digitalWrite(a.in1, HIGH);
    digitalWrite(a.in2, LOW);
    analogWrite(a.enable, map(-velocity, 0, 100, 70, 255));
    
    digitalWrite(b.in1, HIGH);
    digitalWrite(b.in2, LOW);
    analogWrite(b.enable, map(-velocity, 0, 100, 70, 255));
  }

}

long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}
