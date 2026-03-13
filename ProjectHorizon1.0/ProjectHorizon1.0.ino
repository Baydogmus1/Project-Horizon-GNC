#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ---------------------------------------------------------
// Objects and Variables
// ---------------------------------------------------------
Adafruit_LSM6DSO32 dso32;    // Adafruit LSM6DS Library Use for Accelerometer

// Instantiate Hardware Serial2 for Bluetooth (RX = PA3, TX = PA2)
HardwareSerial Serial2(PA3, PA2);

enum {
  prelaunch,
  launch,
  coast,
  descend
} state = prelaunch;

typedef struct {
  float Q_angle;   // Process noise: Uncertainty in the angle evolution
  float Q_bias;    // Process noise: Uncertainty in the gyro bias
  float R_measure; // Measurement noise: Uncertainty in the accelerometer measure

  float angle;     // The calculated angle (OUTPUT)
  float bias;      // The calculated gyro bias
  float rate;      // Unbiased rate

  float P[2][2];   // Error covariance matrix
} Kalman_t;

// Kalman Filter
Kalman_t KalmanX; // For Roll
Kalman_t KalmanY; // For Pitch
uint32_t timer;

// Calibration Variables
float gyroX_offset = 0;
float gyroY_offset = 0;
float gyroZ_offset = 0;
int num_samples = 500;

// PID Variables
float current_error, previous_error = 0;
float prop_error = 0;
float total_integrated_error, integral_error = 0;
float derivative_error = 0;
float motor_control, motor_speed = 0;

// PID Gains
float Kp = 15;
float Ki = 40;
float Kd = 2;

float launch_thresh = 9.81 * 5; // Threshold for when sensor detects launch, g * Number

// Function Prototypes
float PIDController(float error, float prev_error); 
void LSM6DSO32Setup(); 
void calibrategyro(); 
void Kalman_Init(Kalman_t *kf); 
float Kalman_GetAngle(Kalman_t *kf, float newAngle, float newRate, float dt); 

// ---------------------------------------------------------
// Main Setup
// ---------------------------------------------------------
void setup(void) {
  Serial.begin(115200);  // USB to PC
  
  // Start Hardware Serial2 for Bluetooth 
  Serial2.begin(115200); 

  // Removed the 'while (!Serial)' trap so the rocket can run on battery!
  delay(1000); // Give sensors a moment to power up

  // LSM connections and calibration
  Serial.println("LSM6DSO32 test!");
  LSM6DSO32Setup();
  calibrategyro();

  // Initialize Kalman filter parameters
  Kalman_Init(&KalmanX);
  Kalman_Init(&KalmanY);
  timer = micros(); // Start timer

  Serial.println("");

  //Initialize Pin for motor direction control
  pinMode(B11, OUTPUT);
}

// ---------------------------------------------------------
// Main Loop
// ---------------------------------------------------------
void loop() {
  sensors_event_t accel, gyro, temp;
  unsigned long last_time = millis();
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

  current_error = (gyro.gyro.z - gyroZ_offset) * 57.29578f;

  switch(state){
    case prelaunch:
      if(millis() - last_time > 1000)
      {
        // 2. Print Acceleration Data to Bluetooth
        Serial2.print("Acc[X,Y,Z]: [");
        Serial2.print(accel.acceleration.x); Serial2.print(", ");
        Serial2.print(accel.acceleration.y); Serial2.print(", ");
        Serial2.print(accel.acceleration.z); Serial2.print("]\t");
        Serial2.print("Mag: "); Serial2.print(accel_mag); Serial2.print("\t");

        // 3. Print Orientation & Temp to Bluetooth
        Serial2.print("Temp: "); Serial2.print(temp.temperature); Serial2.print("C\t");
        Serial2.print("Roll: "); Serial2.print(roll); Serial2.print("\t");
        Serial2.println("Pitch: "); Serial2.print(pitch); Serial2.print("\t");
      }

      // If in prelaunch, gravity magnitude should be ~9.81. If it's off by more than 2 m/s^2, flag it.
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
    break;
    
    case descend:
    break;
  }

  if(state == launch){

  }

  if(state == coast) {
    motor_control = PIDController(current_error, previous_error);

      if (motor_control < 1) {
        digitalWrite(B11, HIGH);
      }       
    else digitalWrite(B11, LOW);

    //mapping motor_control value to PWM duty cycle
    motor_speed = map(abs(motor_control), 0, 32767, 0, 255);
    analogWrite(A0, motor_speed);
  }

  //Save current error for derivative use
  previous_error = current_error;
}

// ---------------------------------------------------------
// Helper Functions 
// ---------------------------------------------------------
void LSM6DSO32Setup() {
  if (!dso32.begin_I2C()) {
    Serial.println("Failed to find LSM6DSO32 chip");
    while (1) {
      Serial.println("Failed to find LSM6DSO32 chip");
      delay(10);
    }
  }
  Serial.println("LSM6DSO32 Found!");

  dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
  dso32.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  dso32.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  dso32.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
}

void calibrategyro() {
  Serial.println("Keep device still. Calibrating Gyro...");
  float x_sum = 0;
  float y_sum = 0;
  float z_sum = 0;

  for (int i = 0; i < num_samples; i++) {
    sensors_event_t accel, gyro, temp;
    dso32.getEvent(&accel, &gyro, &temp);

    x_sum += gyro.gyro.x;
    y_sum += gyro.gyro.y;
    z_sum += gyro.gyro.z;
    delay(2); 
  }

  gyroX_offset = x_sum / num_samples;
  gyroY_offset = y_sum / num_samples;
  gyroZ_offset = z_sum / num_samples;

  Serial.print("Calibration Complete. Z Offset: ");
  Serial.println(gyroZ_offset);
}

float PIDController(float error, float prev_error) {
  prop_error = Kp * error;
  total_integrated_error += error;
  integral_error = Ki * error;
  derivative_error = Kd * (error - prev_error);

  float control_output = prop_error + integral_error + derivative_error;
  return control_output;
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