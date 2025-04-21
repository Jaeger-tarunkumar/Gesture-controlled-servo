#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servo;

float anglex = 90.0;
unsigned long previous_time = 0;
float delta_time = 0;
float omega = 0;
float gx_bias = 0; // to store initial gyro bias

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  servo.attach(9);

  Serial.println("Initializing MPU6050...");
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  Serial.println("MPU6050 is ready!");

  // Measure gyro bias when stationary
  int32_t gx_sum = 0;
  for (int i = 0; i < 500; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gx_sum += gx;
    delay(2); // slight delay between readings
  }
  gx_bias = gx_sum / 500.0;
  Serial.print("Gyro X bias = ");
  Serial.println(gx_bias);

  previous_time = millis();
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long current_time = millis();
  delta_time = (current_time - previous_time) / 1000.0;
  previous_time = current_time;

  omega = (gx - gx_bias) / 131.0; // Corrected gyro reading

  // Add a small deadband to ignore noise
  if (abs(omega) > 1.0) { // ignore omega if less than 1 deg/sec
    anglex += omega * delta_time;
  }

  // Limit servo motion
  if (anglex > 180) anglex = 180;
  if (anglex < 0) anglex = 0;

  servo.write(anglex);

  Serial.print("Angle X: ");
  Serial.println(anglex);

  delay(20);
}
