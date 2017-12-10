#include "quaternionFilters.h"
#include "MPU9250.h"
#include <Wire.h>
#include <stdint.h>


#define AHRS true         // Set to false for basic data read
#define SerialDebug true


MPU9250 IMU2(NOT_SPI, 1);
//MPU9250 IMU2(NOT_SPI, 2);


void setup() {

  Wire.begin();
  Serial.begin(38400);
  byte c1 = IMU2.readByte(IMU2.getAddr(), WHO_AM_I_MPU9250);
    
  if (c1 == 0x71)
    Serial.println(F("MPU2 is connected!"));
  else
    Serial.println(F("MPU2 is not connected!"));

  if (c1 == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU2 is online..."));
    
    // Start by performing self test and reporting values
    IMU2.MPU9250SelfTest(IMU2.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(IMU2.selfTest[0],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(IMU2.selfTest[1],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(IMU2.selfTest[2],1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(IMU2.selfTest[3],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(IMU2.selfTest[4],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(IMU2.selfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    IMU2.calibrateMPU9250(IMU2.gyroBias, IMU2.accelBias);
    Serial.println("Accelerometer1 biases (g)");
    Serial.println(IMU2.accelBias[0]);
    Serial.println(IMU2.accelBias[1]);
    Serial.println(IMU2.accelBias[2]);

    Serial.println("Gyro1 biases (deg/s)");
    Serial.println(IMU2.gyroBias[0]);
    Serial.println(IMU2.gyroBias[1]);
    Serial.println(IMU2.gyroBias[2]);    

    IMU2.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println(F("MPU1 initialized for active data mode...."));

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d1 = mag_reg_rd(IMU2, WHO_AM_I_AK8963);
    Serial.print(F("Magnetometer1 has address 0x"));
    Serial.println(d1, HEX);
    
    if (d1 != 0x48 )
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }
    

    // Get magnetometer calibration from AK8963 ROM
    //IMU2.initAK8963(IMU2.factoryMagCalibration);
    setup_mag(IMU2, IMU2.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("Magnetometer1 initialized for active data mode....");

    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(IMU2.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(IMU2.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(IMU2.factoryMagCalibration[2], 2);
    }

    // Get sensor resolutions, only need to do this once
    IMU2.getAres();
    IMU2.getGres();
    IMU2.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    //IMU2.magCalMPU9250(IMU2.magBias, IMU2.magScale);
    magCal(IMU2, IMU2.magBias, IMU2.magScale);
    Serial.println("Magnetometer1 mag biases (mG)");
    Serial.println(IMU2.magBias[0]);
    Serial.println(IMU2.magBias[1]);
    Serial.println(IMU2.magBias[2]);

    Serial.println("Magnetometer1 mag scale (mG)");
    Serial.println(IMU2.magScale[0]);
    Serial.println(IMU2.magScale[1]);
    Serial.println(IMU2.magScale[2]);
    delay(2000); // Add delay to see results before serial spew of data

    if(SerialDebug)
    {
      Serial.println("Magnetometer:");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(IMU2.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(IMU2.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(IMU2.factoryMagCalibration[2], 2);
    }

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c1, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }

}

void loop() {
  // put your main code here, to run repeatedly:
    // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (IMU2.readByte(IMU2.getAddr(), INT_STATUS) & 0x01)
  {
    IMU2.readAccelData(IMU2.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    IMU2.ax = (float)IMU2.accelCount[0] * IMU2.aRes;// - IMU2.accelBias[0];
    IMU2.ay = (float)IMU2.accelCount[1] * IMU2.aRes;// - IMU2.accelBias[1];
    IMU2.az = (float)IMU2.accelCount[2] * IMU2.aRes;// - IMU2.accelBias[2];

    IMU2.readGyroData(IMU2.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    IMU2.gx = (float)IMU2.gyroCount[0] * IMU2.gRes;// - IMU2.gyroBias[0];
    IMU2.gy = (float)IMU2.gyroCount[1] * IMU2.gRes;// - IMU2.gyroBias[1];
    IMU2.gz = (float)IMU2.gyroCount[2] * IMU2.gRes;// - IMU2.gyroBias[2];

    readMag(IMU2,IMU2.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    IMU2.mx = (float)IMU2.magCount[0] * IMU2.mRes
               * IMU2.factoryMagCalibration[0] - IMU2.magBias[0];
    IMU2.my = (float)IMU2.magCount[1] * IMU2.mRes
               * IMU2.factoryMagCalibration[1] - IMU2.magBias[1];
    IMU2.mz = (float)IMU2.magCount[2] * IMU2.mRes
               * IMU2.factoryMagCalibration[2] - IMU2.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  IMU2.updateTime();

 // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(IMU2.q, IMU2.eInt, IMU2.ax, IMU2.ay, IMU2.az, IMU2.gx * DEG_TO_RAD,
                         IMU2.gy * DEG_TO_RAD, IMU2.gz * DEG_TO_RAD, IMU2.my,
                         IMU2.mx, IMU2.mz, IMU2.deltat); 

  if (!AHRS)
  {
    IMU2.delt_t = millis() - IMU2.count;
    if (IMU2.delt_t > 500)
    {
      if(SerialDebug)
      {
        // Print acceleration values in milligs!
        Serial.print("X-acceleration: "); Serial.print(1000 * IMU2.ax);
        Serial.print(" mg ");
        Serial.print("Y-acceleration: "); Serial.print(1000 * IMU2.ay);
        Serial.print(" mg ");
        Serial.print("Z-acceleration: "); Serial.print(1000 * IMU2.az);
        Serial.println(" mg ");

        // Print gyro values in degree/sec
        Serial.print("X-gyro rate: "); Serial.print(IMU2.gx, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Y-gyro rate: "); Serial.print(IMU2.gy, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Z-gyro rate: "); Serial.print(IMU2.gz, 3);
        Serial.println(" degrees/sec");

        // Print mag values in degree/sec
        Serial.print("X-mag field: "); Serial.print(IMU2.mx);
        Serial.print(" mG ");
        Serial.print("Y-mag field: "); Serial.print(IMU2.my);
        Serial.print(" mG ");
        Serial.print("Z-mag field: "); Serial.print(IMU2.mz);
        Serial.println(" mG");

        IMU2.tempCount = IMU2.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        IMU2.temperature = ((float) IMU2.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
        Serial.print("Temperature is ");  Serial.print(IMU2.temperature, 1);
        Serial.println(" degrees C");
      }
      IMU2.count = millis();
    } // if (IMU2.delt_t > 500)
  } // if (!AHRS)

  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    IMU2.delt_t = millis() - IMU2.count;

    // update LCD once per half-second independent of read rate
    if (IMU2.delt_t > 500)
    {
      if(SerialDebug)
      {
        Serial.print("ax = ");  Serial.print((int)1000 * IMU2.ax);
        Serial.print(" ay = "); Serial.print((int)1000 * IMU2.ay);
        Serial.print(" az = "); Serial.print((int)1000 * IMU2.az);
        Serial.println(" mg");

        Serial.print("gx = ");  Serial.print(IMU2.gx, 2);
        Serial.print(" gy = "); Serial.print(IMU2.gy, 2);
        Serial.print(" gz = "); Serial.print(IMU2.gz, 2);
        Serial.println(" deg/s");

        Serial.print("mx = ");  Serial.print((int)IMU2.mx);
        Serial.print(" my = "); Serial.print((int)IMU2.my);
        Serial.print(" mz = "); Serial.print((int)IMU2.mz);
        Serial.println(" mG");

        Serial.print("q0 = ");  Serial.print(IMU2.q[0]);
        Serial.print(" qx = "); Serial.print(IMU2.q[1]);
        Serial.print(" qy = "); Serial.print(IMU2.q[2]);
        Serial.print(" qz = "); Serial.println(IMU2.q[3]);
      }

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
       IMU2.yaw   = atan2(2.0 * IMU2.q[1] * IMU2.q[2] + IMU2.q[0] * IMU2.q[3], 
                       IMU2.q[0] * IMU2.q[0] + IMU2.q[1] * IMU2.q[1] - 
                       IMU2.q[2] * IMU2.q[2] - IMU2.q[3] * IMU2.q[3]);
      IMU2.pitch = -asin(2.0 * IMU2.q[1] * IMU2.q[3] - IMU2.q[0]* IMU2.q[2]);
      IMU2.roll  = atan2(2.0 * (IMU2.q[0] * IMU2.q[1]) + (IMU2.q[2]* IMU2.q[3]),
                      IMU2.q[0] * (IMU2.q[0] - IMU2.q[1]) * (IMU2.q[1] - IMU2.q[2])
                      * (IMU2.q[2] + IMU2.q[3]* IMU2.q[3]));
                      
      IMU2.pitch *= RAD_TO_DEG;
      IMU2.yaw   *= RAD_TO_DEG;

      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      IMU2.yaw  -= 8.5;
      IMU2.roll *= RAD_TO_DEG;

      if(SerialDebug)
      {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(IMU2.yaw, 2);
        Serial.print(", ");
        Serial.print(IMU2.pitch, 2);
        Serial.print(", ");
        Serial.println(IMU2.roll, 2);

        Serial.print("rate = ");
        Serial.print((float)IMU2.sumCount / IMU2.sum, 2);
        Serial.println(" Hz");
      }
      IMU2.count = millis();
      IMU2.sumCount = 0;
      IMU2.sum = 0;
    } // if (IMU2.delt_t > 500)
  } // if (AHRS)
  
}

uint8_t mag_reg_rd(MPU9250 mpu,uint8_t reg)
{
  mpu.writeByte(mpu.getAddr(), I2C_SLV0_ADDR, 0x80 | AK8963_ADDRESS);
  mpu.writeByte(mpu.getAddr(), I2C_SLV0_REG, reg);
  mpu.writeByte(mpu.getAddr(), I2C_SLV0_CTRL, 0x81);
  delay(500);
  return mpu.readByte(mpu.getAddr(), EXT_SENS_DATA_00);
}

void mag_reg_wr(MPU9250 mpu,uint8_t reg,uint8_t val)
{
    mpu.writeByte(mpu.getAddr(), I2C_SLV0_ADDR, AK8963_ADDRESS);
    mpu.writeByte(mpu.getAddr(), I2C_SLV0_REG, reg);
    mpu.writeByte(mpu.getAddr(), I2C_SLV0_CTRL, 0x81);
    mpu.writeByte(mpu.getAddr(), I2C_SLV0_DO, val);
    mpu.writeByte(mpu.getAddr(), I2C_SLV0_CTRL, 0x81);
    delay(500);
}

void setup_mag(MPU9250 mpu, float * destination)
{
    //Read calibration
    mag_reg_wr(mpu, AK8963_CNTL, 0x00);
    mag_reg_wr(mpu, AK8963_CNTL, 0x0F);
    uint8_t cal_x = mag_reg_rd(mpu, AK8963_ASAX);
    uint8_t cal_y = mag_reg_rd(mpu, AK8963_ASAY);
    uint8_t cal_z = mag_reg_rd(mpu, AK8963_ASAZ);
    mag_reg_wr(mpu, AK8963_CNTL, 0x00);
    //0x12 for 8Hz, 0x16 for 100Hz
    mag_reg_wr(mpu, AK8963_CNTL, 0x12);

    //Set up auto poll
    mpu.writeByte(mpu.getAddr(), I2C_SLV0_ADDR, 0x80 | AK8963_ADDRESS);
    //Start read from ST1
    mpu.writeByte(mpu.getAddr(), I2C_SLV0_REG, AK8963_ST1);
    //Read 8 Bytes
    mpu.writeByte(mpu.getAddr(), I2C_SLV0_CTRL, 0x88);

    //Manipulate calibration
    destination[0] = (float)(cal_x - 128) / 256.0 + 1.;
    destination[1] = (float)(cal_y - 128) / 256.0 + 1.;
    destination[2] = (float)(cal_z - 128) / 256.0 + 1.;
}

void readMag(MPU9250 mpu, int16_t * destination)
{
  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end
  // of data acquisition
  //float t;
  uint8_t rawData[8];

  mpu.readBytes(mpu.getAddr(), EXT_SENS_DATA_00, 8, &rawData[0]);
/*
  for (int i = 0; i<8; i++)
  {
    Serial.print("0x");
    Serial.print(rawData[i], HEX);
    Serial.print(' ');
  }
  Serial.println(); */
  // Wait for magnetometer data ready bit to be set
  //if (rawData[0] & 0x01)
  //{
    // Read the six raw data and ST2 registers sequentially into data array

    
    uint8_t c = rawData[7]; // End data read by reading ST2 register
    // Check if magnetic sensor overflow set, if not then report data
    if (!(c & 0x08))
    {
      // Turn the MSB and LSB into a signed 16-bit value
      destination[0] = ((int16_t)rawData[2] << 8) | rawData[1];
      // Data stored as little Endian
      destination[1] = ((int16_t)rawData[4] << 8) | rawData[3];
      destination[2] = ((int16_t)rawData[6] << 8) | rawData[5];
    }
  //}
  /*
  else 
  {
    Serial.println(F("ST1 not setting up correctly!"));
    //count++;
    //Serial.println(count);  
  } */
}


void magCal(MPU9250 mpu, float * bias_dest, float * scale_dest)
{
  uint16_t ii = 0, sample_count = 0;
  float t;
  int32_t mag_bias[3]  = {0, 0, 0},
          mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3]  = {0x8000, 0x8000, 0x8000},
          mag_min[3]  = {0x7FFF, 0x7FFF, 0x7FFF},
          mag_temp[3] = {0, 0, 0};

  // Make sure resolution has been calculated
  mpu.getMres();

  Serial.println(F("Mag Calibration: Wave device in a figure 8 until done!"));
  Serial.println(
      F("  4 seconds to get ready followed by 15 seconds of sampling)"));
  delay(4000);

  // shoot for ~fifteen seconds of mag data
  // at 8 Hz ODR, new mag data is available every 125 ms
/*  if (Mmode == M_8HZ)
  { */
    sample_count = 128;
 /* }
  // at 100 Hz ODR, new mag data is available every 10 ms
  if (Mmode == M_100HZ)
  {
    sample_count = 1500;
  } */

  for (ii = 0; ii < sample_count; ii++)
  { 
    //Serial.println(F("Achived!"));
    //t = millis();
    readMag(mpu, mag_temp);  // Read the mag data
    //Serial.print(F("Read takes: "));
    //Serial.println(millis() - t);

    for (int jj = 0; jj < 3; jj++)
    {
      if (mag_temp[jj] > mag_max[jj])
      {
        mag_max[jj] = mag_temp[jj];
      }
      if (mag_temp[jj] < mag_min[jj])
      {
        mag_min[jj] = mag_temp[jj];
      }
    }
/*
    if (Mmode == M_8HZ)
    { */
      delay(135); // At 8 Hz ODR, new mag data is available every 125 ms
  /*  }
    if (Mmode == M_100HZ)
    {
    
      delay(12);  // At 100 Hz ODR, new mag data is available every 10 ms
    }*/
  }

  // Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
  // Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
  // Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

  // Get hard iron correction
  // Get 'average' x mag bias in counts
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2;
  // Get 'average' y mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2;
  // Get 'average' z mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2;

  // Save mag biases in G for main program
  bias_dest[0] = ((float)mag_bias[0]) * mpu.mRes * mpu.factoryMagCalibration[0];
  bias_dest[1] = ((float)mag_bias[1]) * mpu.mRes * mpu.factoryMagCalibration[1];
  bias_dest[2] = ((float)mag_bias[2])* mpu.mRes * mpu.factoryMagCalibration[2];

  // Get soft iron correction estimate
  // Get average x axis max chord length in counts
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2;
  // Get average y axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2;
  // Get average z axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2;

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  scale_dest[0] = avg_rad / ((float)mag_scale[0]);
  scale_dest[1] = avg_rad / ((float)mag_scale[1]);
  scale_dest[2] = avg_rad / ((float)mag_scale[2]);

  Serial.println(F("Mag Calibration done!"));
}





