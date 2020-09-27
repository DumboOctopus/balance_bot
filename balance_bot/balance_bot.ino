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
double Kp= 2, Ki=0.1, Kd=3;

float previous_state[2] = {0,0};
float prev_error[2][2];
float Q_angle = 0.001;
float Q_gyroBias = 0.003;
float R_measure = 0.03;
float angle = 0;

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

  // kalman filter
  prev_error[0][0] = 0;
  prev_error[0][1] = 0;
  prev_error[1][0] = 0;
  prev_error[1][1] = 0;
  
}

void loop() {
  
  long duration, inches, cm;
  cm = read_ultrasonic(ultra_forward);

  unsigned long currentMillis = millis();
  unsigned long elapsedTime = currentMillis - previousMillis;


  if (elapsedTime >= DELAY) {
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

    //pitch += (double)(180/3.14 *g.gyro.x * elapsedTime /1000.0);
    float reading = 180/3.14 * g.gyro.x;
    // kalman filter
    double dt = elapsedTime / 1000.0;
    
    float priori_state[2];
    priori_state[0] = previous_state[0] + dt*(reading - previous_state[1]);

    priori_state[1] = previous_state[1]; // we can't really predict the future bias from measurements

    // estimate priori covariance
    float priori_error[2][2];
    priori_error[0][0] += dt * (
      dt*prev_error[1][1] - prev_error[0][1] - prev_error[1][0] + Q_angle
    );
    priori_error[0][1] -= dt * prev_error[1][1];
    priori_error[1][0] -= dt * prev_error[1][1];
    priori_error[1][1] += Q_gyroBias * dt;

    float y = reading - priori_state[0];

    float S = priori_error[0][0] + R_measure;

    float kalman_gain[2] = {priori_error[0][0]/S, priori_error[1][0]/S};


    // update the previous state
    // now previous_state contains the latest state information
    
    previous_state[0] += kalman_gain[0]*y;
    previous_state[1] += kalman_gain[1]*y;

    float P00_temp = prev_error[0][0];
    float P01_temp = prev_error[0][1];
    
    prev_error[0][0] -= kalman_gain[0] * P00_temp;
    prev_error[0][1] -= kalman_gain[0] * P01_temp;
    prev_error[1][0] -= kalman_gain[1] * P00_temp;
    prev_error[1][1] -= kalman_gain[1] * P01_temp;

    // result
    angle += previous_state[0]*dt;
    // done with kalman filter
#if 1
    Serial.print("angle ");
    Serial.print(angle);
    Serial.print("g ");

    Serial.print(g.gyro.x);
    Serial.println();
#endif 
    
    // PID control
    double error = angle;
    double cumError = error * elapsedTime;
    double rateError = (error - lastError)/elapsedTime;
    int output = (int)(Kp * error + Ki * cumError + Kd * rateError);
    lastError = error;

   
    set_motor_speed(output, right_motor, left_motor);

#if 0
    Serial.print("Rotation X: ");
    Serial.print(pitch);
    Serial.print(", Output: ");
    Serial.print(output);
    Serial.println(" rad");
#endif 
#if 0
    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");
#endif
#if 0
  Serial.print("Loop time ");
  Serial.println(millis() - currentMillis);

#endif

      previousMillis = currentMillis;
  }
  


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
    velocity += 60;
    digitalWrite(a.in1, LOW);
    digitalWrite(a.in2, HIGH);
    analogWrite(a.enable, min(velocity, 255));
    
    digitalWrite(b.in1, LOW);
    digitalWrite(b.in2, HIGH);
    analogWrite(b.enable, min(velocity, 255));
  } else {
    velocity -= 60;
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
