#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h> 

Adafruit_LSM6DSO32 dso32;

Servo esc;
const int ESC_PIN = PA0;
const int ESC_REVERSE_MAX = 1000;
const int ESC_NEUTRAL     = 1500;
const int ESC_FORWARD_MAX = 2000;

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
float Ki = 0; //-5.9426;
float Kd = 0;
const float MAX_PID_OUTPUT = 12.0f; //Battery Voltage

// Prototypes
float PIDController(float error, float prev_error, double dt);
void LSM6DSO32Setup(); 
void calibrategyro(); 
void Kalman_Init(Kalman_t *kf); 
float Kalman_GetAngle(Kalman_t *kf, float newAngle, float newRate, float dt); 
void armESC(); 
int sendESCPWM(float control_signal); // Updated to return the PWM value

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void setup(void) {
  Serial.begin(115200);
  
  // Explicitly set the RX and TX pins for the built-in Serial2
  Serial2.setRx(PA3);
  Serial2.setTx(PA2);
  Serial2.begin(9600); // Set to 9600 to match default HC-05 Data Mode

  // Wait for USB Serial to connect so you don't miss the boot sequence
  uint32_t timeout = millis();
  while (!Serial && (millis() - timeout < 3000)) {}

  Serial.println("System Booting...");

  // Attach and Arm the physical ESC
  esc.attach(ESC_PIN, ESC_REVERSE_MAX, ESC_FORWARD_MAX);
  Serial.println("Arming ESC...");
  armESC();

  LSM6DSO32Setup();
  Serial.println("Sensor Initialized.");
  
  Serial.println("Calibrating Gyro... Do not move the sensor!");
  calibrategyro();
  Serial.println("Calibration Complete.");

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
  
  // Send the signal to the ESC and grab the resulting pulse width
  int current_pwm = sendESCPWM(motor_control);

  // Telemetry output to both USB Serial and Bluetooth Serial2
  // Set to 100ms (10Hz) to prevent buffer overflows on 9600 baud Bluetooth
  if(millis() - last_print > 100) { 
    
    // --- USB Serial Output (PC) ---
    Serial.print("RErr:");   Serial.print(current_error, 2);
    Serial.print("\tV:");    Serial.print(motor_control, 2);
    Serial.print("\tPWM:");  Serial.print(current_pwm);
    Serial.print("\tR:");    Serial.print(roll, 2);
    Serial.print("\tP:");    Serial.println(pitch, 2);
    
    // --- Bluetooth Serial2 Output (Phone) ---
    Serial2.print("RErr:");  Serial2.print(current_error, 2);
    Serial2.print("\tV:");   Serial2.print(motor_control, 2);
    Serial2.print("\tPWM:"); Serial2.print(current_pwm);
    Serial2.print("\tR:");   Serial2.print(roll, 2);
    Serial2.print("\tP:");   Serial2.println(pitch, 2);
    
    last_print = millis();
  }

  previous_error = current_error;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void armESC() {
  esc.writeMicroseconds(ESC_NEUTRAL);
  delay(2000); 
}

int sendESCPWM(float control_signal) {
  // Deadband: Prevents the motor from jittering/humming when near zero
  if (abs(control_signal) < 0.05) {
    esc.writeMicroseconds(ESC_NEUTRAL);
    return ESC_NEUTRAL;
  }
  
  float throttle_ratio = control_signal / MAX_PID_OUTPUT; // Result is -1.0 to 1.0
  int pulseWidth = ESC_NEUTRAL + (int)(throttle_ratio * 500);
  
  // Safety constraint
  pulseWidth = constrain(pulseWidth, ESC_REVERSE_MAX, ESC_FORWARD_MAX);
  
  // Write to the actual hardware pin
  esc.writeMicroseconds(pulseWidth);
  
  // Return the value for telemetry printing
  return pulseWidth;
}

float PIDController(float error, float prev_error, double dt) {
  if (dt <= 0.0) return 0.0;

  prop_error = Kp * error;

  total_integrated_error += (error * dt);
  
  if (Ki != 0) {
      float max_accum = abs(MAX_PID_OUTPUT / Ki); 
      total_integrated_error = constrain(total_integrated_error, -max_accum, max_accum);
      integral_error = Ki * total_integrated_error;
  } else {
      integral_error = 0;
      total_integrated_error = 0;
  }

  derivative_error = Kd * ((error - prev_error) / dt);

  float control_output = prop_error + integral_error + derivative_error;
  return constrain(control_output, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
}

void LSM6DSO32Setup() {
  // Force the I2C pins BEFORE calling begin_I2C()
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();

  if (!dso32.begin_I2C()){
    while(1){
      Serial.println("I2C Not Found");
      delay(500);
    }
  }
  
  dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
  dso32.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  dso32.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
  dso32.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
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