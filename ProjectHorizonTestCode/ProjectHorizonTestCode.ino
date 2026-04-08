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

typedef struct {
  float Q_angle, Q_bias, R_measure;
  float angle, bias, rate;
  float P[2][2];
} Kalman_t;

Kalman_t KalmanX, KalmanY;
uint32_t timer;

float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
int num_samples = 500;

float current_error, previous_error = 0;
float prop_error, integral_error, derivative_error, total_integrated_error = 0;
float motor_control = 0; 

// MATLAB GAINS
float Kp = -2.5873;
float Ki = -5.9426;
float Kd = 0;
const float MAX_PID_OUTPUT = 12.0f; 

// Prototypes
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

  pinMode(DIR_PIN, OUTPUT);
  
  esc.attach(ESC_PIN, ESC_MIN_THROTTLE, ESC_MAX_THROTTLE);
  armESC();

  LSM6DSO32Setup();
  calibrategyro();

  Kalman_Init(&KalmanX);
  Kalman_Init(&KalmanY);
  timer = micros();
}

void loop() {
  sensors_event_t accel, gyro, temp;
  static unsigned long last_print = 0;
  
  double dt = (double)(micros() - timer) / 1000000.0;
  timer = micros();

  dso32.getEvent(&accel, &gyro, &temp);

  // Unit conversion for Kalman (Degrees)
  float gyroRateX = (gyro.gyro.x - gyroX_offset) * 57.29578f;
  float gyroRateY = (gyro.gyro.y - gyroY_offset) * 57.29578f;
  float accRoll   = atan2(accel.acceleration.y, accel.acceleration.z) * 57.29578f;
  float accPitch  = atan2(-accel.acceleration.x, sqrt(pow(accel.acceleration.y,2) + pow(accel.acceleration.z, 2))) * 57.29578f;

  float roll  = Kalman_GetAngle(&KalmanX, accRoll, gyroRateX, dt);
  float pitch = Kalman_GetAngle(&KalmanY, accPitch, gyroRateY, dt);

  // Rate Control Input (Radians/sec)
  current_error = (gyro.gyro.z - gyroZ_offset);

  motor_control = PIDController(current_error, previous_error, dt);
  
  digitalWrite(DIR_PIN, (motor_control < 0) ? HIGH : LOW);
  sendESCPWM(motor_control);

  // Bluetooth Telemetry
  if(millis() - last_print > 500) {
    Serial2.print("Rate Error: "); Serial2.print(current_error);
    Serial2.print("\tOut Volts: "); Serial2.print(motor_control);
    Serial2.print("\tRoll: "); Serial2.print(roll, 2);
    Serial2.print("Pitch: ");  Serial2.println(pitch, 2);
    last_print = millis();
  }

  previous_error = current_error;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void armESC() {
  esc.writeMicroseconds(ESC_MIN_THROTTLE);
  delay(2000); 
}

void sendESCPWM(float control_signal) {
  // Deadband: If error is tiny, don't hum/spin the motor
  if (abs(control_signal) < 0.1) {
    esc.writeMicroseconds(ESC_MIN_THROTTLE);
    return;
  }

  float clamped_volts = constrain(abs(control_signal), 0.0f, MAX_PID_OUTPUT);
  float throttle_percent = clamped_volts / MAX_PID_OUTPUT;
  int pulseWidth = ESC_MIN_THROTTLE + (int)(throttle_percent * (ESC_MAX_THROTTLE - ESC_MIN_THROTTLE));
  
  esc.writeMicroseconds(pulseWidth);
}

float PIDController(float error, float prev_error, double dt) {
  if (dt <= 0.0) return 0.0;

  prop_error = Kp * error;

  total_integrated_error += (error * dt);
  float max_accum = abs(MAX_PID_OUTPUT / Ki); 
  total_integrated_error = constrain(total_integrated_error, -max_accum, max_accum);
  integral_error = Ki * total_integrated_error;  

  derivative_error = Kd * ((error - prev_error) / dt);

  float control_output = prop_error + integral_error + derivative_error;
  return constrain(control_output, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
}

void LSM6DSO32Setup() {
  if (!dso32.begin_I2C()) while (1); 
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
    delay(5); 
  }
  gyroX_offset = x_sum / num_samples;
  gyroY_offset = y_sum / num_samples;
  gyroZ_offset = z_sum / num_samples;
}

void Kalman_Init(Kalman_t *kf) {
    kf->Q_angle = 0.001f; kf->Q_bias = 0.003f; kf->R_measure = 0.03f;
    kf->angle = 0.0f; kf->bias = 0.0f;
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
    K[0] = kf->P[0][0] / S; K[1] = kf->P[1][0] / S;
    float y = newAngle - kf->angle;
    kf->angle += K[0] * y; kf->bias += K[1] * y;
    float P00_temp = kf->P[0][0]; float P01_temp = kf->P[0][1];
    kf->P[0][0] -= K[0] * P00_temp; kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp; kf->P[1][1] -= K[1] * P01_temp;
    return kf->angle;
}