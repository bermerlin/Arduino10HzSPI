/***************************************************************************
*
* Copyright (C) 2026 www.sailboatinstruments.blogspot.com
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
* copies of the Software, and to permit persons to whom the Software is furnished
* to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
* IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
****************************************************************************/

#include <SPI.h>
#include <LSM6DSOSensor.h>
#include <LIS3MDLSensor.h>

// SPI CS pins
#define LSM_CS   10
#define LIS_CS   4

// Sensor objects
LSM6DSOSensor AccGyr(&SPI, LSM_CS);
LIS3MDLSensor Magneto(&SPI, LIS_CS);

// =============================
// Configuration
// =============================

#define LOOP_HZ        50.0f
#define LOOP_DT        (1.0f / LOOP_HZ)
#define MAG_DIVIDER    5      // 50Hz / 5 = 10Hz mag update

// Complementary gains
#define TILT_GYRO_GAIN 0.98f
#define TILT_ACC_GAIN  (1.0f - TILT_GYRO_GAIN)

// Optional heading smoothing
#define HEADING_ALPHA  0.1f   // 0 = disabled

// --- RMS smoothing time constant (~1 sec at 50 Hz)
#define RMS_ALPHA 0.02f

float gyroRMS2 = 0.0f;    // squared RMS accumulator
float accRMS2  = 0.0f;

enum SeaState {
  SEA_CALM,
  SEA_MODERATE,
  SEA_HEAVY
};

SeaState seaState = SEA_CALM;


// =============================
// Biquad Filter Structure
// =============================

typedef struct {
  float b0, b1, b2;
  float a1, a2;
  float x1, x2;
  float y1, y2;
} Biquad;

float biquad_process(Biquad *f, float x)
{
  float y = f->b0 * x
          + f->b1 * f->x1
          + f->b2 * f->x2
          - f->a1 * f->y1
          - f->a2 * f->y2;

  f->x2 = f->x1;
  f->x1 = x;
  f->y2 = f->y1;
  f->y1 = y;

  return y;
}

// 0.6 Hz Butterworth @ 10 Hz
Biquad magFilterX = {
  0.02786f, 0.05572f, 0.02786f,
 -1.47548f, 0.58692f,
  0,0,0,0
};

Biquad magFilterY = {
  0.02786f, 0.05572f, 0.02786f,
 -1.47548f, 0.58692f,
  0,0,0,0
};

float shortestAngle(float A, float B)
{
    float delta = A - B;
    delta = fmod(delta + 180.0, 360.0);
    if (delta < 0)
        delta += 360.0;
    return delta - 180.0;
}

// =============================
// Global State
// =============================

float roll = 0.0f;
float pitch = 0.0f;
float heading_smoothed = 0.0f;

int magCounter = 0;

// Buffer for accelerometer readings.
int16_t reads[3];

// Buffer for magnetometer readings.
int16_t readsm[3];

// Buffer for gyroscope readings.
int16_t readsg[3];

// Magnetometer calibration data
float m_xBias, m_sens_x[3];
float m_yBias, m_sens_y[3];
float m_zBias, m_sens_z[3];

// Accelerometer calibration data
float a_xBias, sens_x[3];
float a_yBias, sens_y[3];
float a_zBias, sens_z[3];

float g_xBias, g_yBias, g_zBias;

float earthField = 0.5f;     // auto-calibrated after startup  (gauss)
float prevXh = 0.0f;
float prevYh = 0.0f;

bool magValid = true;
float yaw_gyro = 0.0f;       // yaw from gyro integration

bool startupAligned = false;
unsigned long startupTime;

// =============================
// Sensor Reads
// =============================

void readIMU(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz)
{

  AccGyr.Get_X_AxesRaw(reads);

  float rx = reads[0] - a_xBias;
	float ry = reads[1] - a_yBias;
	float rz = reads[2] - a_zBias;
		 			
	ax = sens_x[0] * rx + sens_x[1] * ry + sens_x[2] * rz;
	ay = sens_y[0] * rx + sens_y[1] * ry + sens_y[2] * rz;
	az = sens_z[0] * rx + sens_z[1] * ry + sens_z[2] * rz;

  ax *= 0.122f;  // mg
  ay *= 0.122f;
  az *= 0.122f; 

  AccGyr.Get_G_AxesRaw(readsg);
  // For FS = 250dps (Sensitivity = 8.75 mdps/LSB)
  gx = readsg[0] * 0.00875f * 0.01745329251f;  // rad/s
  gy = readsg[1] * 0.00875f * 0.01745329251f;
  gz = readsg[2] * 0.00875f * 0.01745329251f;

}

void readMag(float &mx, float &my, float &mz)
{
  Magneto.GetAxesRaw(readsm);

  float mrx = readsm[0] - m_xBias;
	float mry = readsm[1] - m_yBias;
	float mrz = readsm[2] - m_zBias;
			
	mx = m_sens_x[0] * mrx + m_sens_x[1] * mry + m_sens_x[2] * mrz;
	my = m_sens_y[0] * mrx + m_sens_y[1] * mry + m_sens_y[2] * mrz;
	mz = m_sens_z[0] * mrx + m_sens_z[1] * mry + m_sens_z[2] * mrz;
}

float smoothAngle(float current, float target, float alpha)
{
    float error = target - current;

    if (error > 180)  error -= 360;
    if (error < -180) error += 360;

    current += alpha * error;

    if (current >= 360) current -= 360;
    if (current < 0)    current += 360;

    return current;
}

// =============================

void setup()
{
  Serial.begin(115200);
  delay(1000);
   
  pinMode(LSM_CS, OUTPUT); digitalWrite(LSM_CS, HIGH);
  pinMode(LIS_CS, OUTPUT); digitalWrite(LIS_CS, HIGH);
  SPI.begin();

  a_xBias = 0.0f;
  a_yBias = 0.0f;
  a_zBias = 0.0f;

  sens_x[0] = 1.0f;
  sens_x[1] = 0.0f;
  sens_x[2] = 0.0f;
   
  sens_y[0] = 0.0f;
  sens_y[1] = 1.0f;
  sens_y[2] = 0.0f;
   
  sens_z[0] = 0.0f;
  sens_z[1] = 0.0f;
  sens_z[2] = 1.0f;

  g_xBias = 8.0f;
  g_yBias = -9.0f;
  g_zBias = -4.0f;
 
  m_xBias = -3184.5f;
  m_yBias = -1822.4f;
  m_zBias = -1078.7f;

  m_sens_x[0] = 1.0f;
  m_sens_x[1] = 0.0f;
  m_sens_x[2] = 0.0f;
   
  m_sens_y[0] = 0.0f;
  m_sens_y[1] = 1.0f;
  m_sens_y[2] = 0.0f;
   
  m_sens_z[0] = 0.0f;
  m_sens_z[1] = 0.0f;
  m_sens_z[2] = 1.0f;

  // Configure and enable accelerometer
  AccGyr.begin();
  AccGyr.Write_Reg(0x17, 0x40);     // 01000000 LPF2 bandwith  ODR/20
  //AccGyr.Write_Reg(0x17, 0x20);   // 00100000 LPF2 bandwith  ODR/10
  AccGyr.Write_Reg(0x10, 0x3A);  //  00111010  ODR 52 Hz, full-scale 4g, LPF2 enabled
  
  // Configure and enable gyroscope
  AccGyr.Write_Reg(0x13, 0x02);      // Enable LPF1 filter
  AccGyr.Write_Reg(0x15, 0x07);      // LPF1 9.7 Hz cutoff frequency for 52 Hz ODR
  // The digital LPF2 filter is always enabled and cannot be configured by the user; its cutoff frequency depends on the selected gyroscope ODR (16.6 Hz for 52 Hz ODR)
  AccGyr.Write_Reg(0x11, 0x30);  //  00110000  ODR 52 Hz, full-scale 250 dps
  
  // Configure and enable magnetometer
  Magneto.begin(); 
  Magneto.WriteReg(0x21, 0x00);   //  4 gauss full-scale
  Magneto.WriteReg(0x23, 0x08);   // set Z-axis mode to High-performance
  Magneto.WriteReg(0x20, 0x50);   // 01010000   High-performance mode, 10 Hz ODR
  Magneto.WriteReg(0x24, 0x40);   // 01000000     BDU
  Magneto.WriteReg(0x22, 0x00);   //  Continuous conversion mode

  delay(250);
  startupTime = millis();

}

// =============================

void loop()
{
  static unsigned long lastMicros = micros();

  // Maintain 50Hz loop timing
  while ((micros() - lastMicros) < (1000000.0f / LOOP_HZ));
  lastMicros += (1000000.0f / LOOP_HZ);

  float ax, ay, az;
  float gx, gy, gz;

  readIMU(ax, ay, az, gx, gy, gz);

  // --- Gyro magnitude squared
  float gyroMag2 = gx*gx + gy*gy + gz*gz;

  // Exponential RMS
  gyroRMS2 = (1.0f - RMS_ALPHA) * gyroRMS2 + RMS_ALPHA * gyroMag2;

  // --- Acceleration deviation
  float accNorm = sqrtf(ax*ax + ay*ay + az*az);
  float accErr = accNorm / 1000.0f - 1.0f;

  accRMS2 = (1.0f - RMS_ALPHA) * accRMS2 + RMS_ALPHA * (accErr * accErr);

  float gyroRMS = sqrtf(gyroRMS2);
  float accRMS  = sqrtf(accRMS2);

  if (gyroRMS < 0.05f && accRMS < 0.02f)
  {
    seaState = SEA_CALM;
  }
  else if (gyroRMS < 0.15f && accRMS < 0.05f)
  {
    seaState = SEA_MODERATE;
  }
  else
  {
    seaState = SEA_HEAVY;
  }

  float adaptiveGain;

  switch (seaState)
  {
    case SEA_CALM:
      adaptiveGain = 0.025f;   // tighter lock
      break;

    case SEA_MODERATE:
      adaptiveGain = 0.015f;   // balanced
      break;

    case SEA_HEAVY:
      adaptiveGain = 0.006f;   // gyro dominant
      break;
  }

  /*
  float motionIndex = 5.0f * gyroRMS + 10.0f * accRMS;

  if (motionIndex > 1.0f)
      motionIndex = 1.0f;

  float adaptiveGain = 0.025f * (1.0f - motionIndex) + 0.005f;
  //Serial.println(adaptiveGain, 3);
  */

  // =============================
  // Complementary Tilt Fusion
  // =============================

  roll  += gx * LOOP_DT;
  pitch += gy * LOOP_DT;

  float roll_acc  = atan2f(ay, az);
  float pitch_acc = atan2f(ax, sqrtf(ay*ay + az*az));

   roll  = TILT_GYRO_GAIN * roll  + TILT_ACC_GAIN * roll_acc;
  pitch = TILT_GYRO_GAIN * pitch + TILT_ACC_GAIN * pitch_acc;

  yaw_gyro -= gz * LOOP_DT * 57.29578f;

  // =============================
  // Magnetometer @ 10Hz
  // =============================

  magCounter++;
  if (magCounter >= MAG_DIVIDER)
  {
  
    magCounter = 0;

    float mx, my, mz;
    readMag(mx, my, mz);

    float mxg, myg, mzg;
    mxg = mx / 6842.0f;
    myg = my / 6842.0f;
    mzg = mz / 6842.0f;

    // ----- Magnitude -----
    float magNorm = sqrtf(mxg*mxg + myg*myg + mzg*mzg);
  
    // Auto-learn Earth field slowly
    earthField = 0.99f * earthField + 0.01f * magNorm;

    // ----- Tilt Compensation -----
    float cosR = cosf(roll);
    float sinR = sinf(roll);
    float cosP = cosf(pitch);
    float sinP = sinf(pitch);

    float Xh = mx * cosP + mz * sinP;
    float Yh = mx * sinR * sinP
            + my * cosR
            - mz * sinR * cosP;

    // ----- Anomaly Detection -----

    bool magnitudeOK = (magNorm > earthField * 0.8f) &&
                       (magNorm < earthField * 1.2f);

    float dX = Xh - prevXh;
    float dY = Yh - prevYh;
    float horizontalJump = sqrtf(dX*dX + dY*dY);

    bool jumpOK = horizontalJump < (0.15f * 6842.0f);  // 0.15f is in gauss units, but the jump is calculated in raw units
    magValid = magnitudeOK && jumpOK;

    // Save for next time
    prevXh = Xh;
    prevYh = Yh;

    if (magValid)
    {
      float Xf = biquad_process(&magFilterX, Xh);
      float Yf = biquad_process(&magFilterY, Yh);

      float heading_mag = atan2f(Yf, Xf) * 57.29578f;
      if (heading_mag < 0) heading_mag += 360.0f;

      // One-time startup alignment after 2 seconds
      
      if (!startupAligned)
      {
        if (millis() - startupTime > 2000 && magValid)
        {
          yaw_gyro = heading_mag;
          heading_smoothed = heading_mag;

          startupAligned = true;
          Serial.println("Startup alignment complete");

          // Disable future time checking
          startupTime = 0;
        }
      }
    
      float yaw_gyro_before = yaw_gyro;
      float correction = shortestAngle(heading_mag, yaw_gyro_before);
      //float yaw_gyro_after = fmod(yaw_gyro_before + 0.02f * correction, 360.0f);
      float yaw_gyro_after = fmod(yaw_gyro_before + adaptiveGain * correction, 360.0f);
      if(yaw_gyro_after < 0.0f) yaw_gyro_after += 360.0f;
      yaw_gyro = yaw_gyro_after;
      
      /*
      Serial.print(heading_mag);
      Serial.print("   ");
      Serial.print(yaw_gyro_before);
      Serial.print("   ");
      Serial.print(yaw_gyro_after);
      Serial.print("   ");
      Serial.print(correction);
      Serial.print("   ");
      */
    }
       
    // ---- Optional smoothing ----
    //heading_smoothed = smoothAngle(heading_smoothed, yaw_gyro, 0.1);
    //Serial.println(heading_smoothed);
    Serial.println(yaw_gyro);

  }
}
