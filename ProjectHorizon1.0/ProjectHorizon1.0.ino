#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h> 

Adafruit_LSM6DSO32 dso32;
HardwareSerial Serial2(PA3, PA2);

Servo esc;
const int ESC_PIN = PA0;
const int DIR_PIN = PB11;
const int ESC_MIN_THROTTLE = 1000;
const int ESC_MAX_THROTTLE = 2000;

enum { prelaunch, launch, coast, descend } state = prelaunch;

typedef struct {
  float Q_angle;
  float Q_bias;
  float R_measure;
  float angle;
  float bias;
  float rate;
  float P[2][2];
} Kalman_t;

Kalman_t KalmanX;
Kalman_t KalmanY;
uint32_t timer;

float gyroX_offset = 0;
float gyroY_offset = 0;
float gyroZ_offset = 0;
int num_samples = 500;

float current_error, previous_error = 0;
float prop_error = 0;
float total_integrated_error, integral_error = 0; // derivative_error = 0;
float motor_control = 0; 

float Kp = -2.5873;
float Ki = -5.9426;
float Kd = 0;
const float MAX_PID_OUTPUT = 12.0f; //Battery Voltage

float launch_thresh = 9.81 * 1.5;

float PIDController(float error, float prev_error, double dt);
void LSM6DSO32Setup(); 
void calibrategyro(); 
void Kalman_Init(Kalman_t *kf); 
float Kalman_GetAngle(Kalman_t *kf, float newAngle, float newRate, float dt); 
void armESC(); 
void sendESCPWM(float control_signal); 

void setup(void) {
  Serial.begin(115200);
  Serial2.begin(115200); 

  esc.attach(ESC_PIN, ESC_MIN_THROTTLE, ESC_MAX_THROTTLE);
  armESC();

  delay(100); 

  LSM6DSO32Setup();
  calibrategyro();

  Kalman_Init(&KalmanX);
  Kalman_Init(&KalmanY);
  timer = micros();

  pinMode(DIR_PIN, OUTPUT);
}

void loop() {
  sensors_event_t accel, gyro, temp;
  static unsigned long last_time = 0;
  double dt = (double)(micros() - timer) / 1000000.0;
  timer = micros();

  dso32.getEvent(&accel, &gyro, &temp);

  float gyroRateX = (gyro.gyro.x - gyroX_offset) * 57.29578f;
  float gyroRateY = (gyro.gyro.y - gyroY_offset) * 57.29578f;

  float accel_mag = sqrt(pow(accel.acceleration.x,2) + pow(accel.acceleration.y,2) + pow(accel.acceleration.z,2));
  float accRoll  = atan2(accel.acceleration.y, accel.acceleration.z) * 57.29578f;
  float accPitch = atan2(-accel.acceleration.x, sqrt(pow(accel.acceleration.y,2) + pow(accel.acceleration.z, 2))) * 57.29578f;

  float roll  = Kalman_GetAngle(&KalmanX, accRoll, gyroRateX, dt);
  float pitch = Kalman_GetAngle(&KalmanY, accPitch, gyroRateY, dt);

  current_error = (gyro.gyro.z - gyroZ_offset);

  switch(state){
    case prelaunch:
      if(millis() - last_time > 1000) {
        Serial2.print("Acc[X,Y,Z]: [");
        Serial2.print(accel.acceleration.x); Serial2.print(", ");
        Serial2.print(accel.acceleration.y); Serial2.print(", ");
        Serial2.print(accel.acceleration.z); Serial2.print("]\t");
        Serial2.print("Mag: "); Serial2.print(accel_mag); Serial2.print("\t");

        Serial2.print("Temp: "); Serial2.print(temp.temperature); Serial2.print("C\t");
        Serial2.print("Roll: "); Serial2.print(roll); Serial2.print("\t");
        Serial2.println("Pitch: "); Serial2.print(pitch); Serial2.print("\t");
        last_time = millis();
      }

      if (state == prelaunch && abs(accel_mag - 9.81) > 2.0) {
       Serial2.print("[FLAG: BAD_PAD_CALIBRATION] ");
      }

      if(accel_mag >= launch_thresh){
        state = launch;
      }
    break;
    
    case launch:
    break;

    case coast:
      motor_control = PIDController(current_error, previous_error, dt);
      if (motor_control < 0) {
        digitalWrite(DIR_PIN, HIGH);
      } else {
        digitalWrite(DIR_PIN, LOW);
      }
      sendESCPWM(motor_control);
    break;
    
    case descend:
    break;
  }

  previous_error = current_error;
}

void armESC() {
  esc.writeMicroseconds(ESC_MIN_THROTTLE);
  delay(1000); 
}

void sendESCPWM(float control_signal) {
  float clamped_volts = constrain(abs(control_signal), 0.0f, MAX_PID_OUTPUT);
  
  // Convert voltage to a percentage, then map to ESC range
  float throttle_percent = clamped_volts / MAX_PID_OUTPUT;
  int pulseWidth = ESC_MIN_THROTTLE + (throttle_percent * (ESC_MAX_THROTTLE - ESC_MIN_THROTTLE));
  
  esc.writeMicroseconds(pulseWidth);
}

void LSM6DSO32Setup() {
  if (!dso32.begin_I2C()) {
    while (1); 
  }

  dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
  dso32.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  dso32.setAccelDataRate(LSM6DS_RATE_833_HZ);
  dso32.setGyroDataRate(LSM6DS_RATE_833_HZ);
}

void calibrategyro() {
  float x_sum = 0, y_sum = 0, z_sum = 0;

  for (int i = 0; i < num_samples; i++) {
  sensors_event_t accel, gyro, temp;
  dso32.getEvent(&accel, &gyro, &temp);

  x_sum += gyro.gyro.x;
  y_sum += gyro.gyro.y;
  z_sum += gyro.gyro.z;
  
  delay(10); // Wait 10ms for a new reading
  }

  gyroX_offset = x_sum / num_samples;
  gyroY_offset = y_sum / num_samples;
  gyroZ_offset = z_sum / num_samples;
}

float PIDController(float error, float prev_error, double dt) {
  prop_error = Kp * error;

  total_integrated_error += (error * dt);
  float max_accum = abs(MAX_PID_OUTPUT / Ki); 
  total_integrated_error = constrain(total_integrated_error, -max_accum, max_accum);
  integral_error = Ki * total_integrated_error;  

  //derivative_error = Kd * ((error - prev_error) / dt);

  float control_output = prop_error + integral_error + derivative_error;
  
  return constrain(control_output, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
}

void Kalman_Init(Kalman_t *kf) {
    kf->Q_angle = 0.001f;
    kf->Q_bias = 0.003f;
    kf->R_measure = 0.03f;
    kf->angle = 0.0f;
    kf->bias = 0.0f;
    kf->P[0][0] = 0.0f; kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f; kf->P[1][1] = 0.0f;
}

float Kalman_GetAngle(Kalman_t *kf, float newAngle, float newRate, float dt) {
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;

    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    float S = kf->P[0][0] + kf->R_measure;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    float y = newAngle - kf->angle;

    kf->angle += K[0] * y;
    kf->bias  += K[1] * y;

    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}