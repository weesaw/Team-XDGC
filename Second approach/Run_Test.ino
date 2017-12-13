#include <secondApproach.h>
#include "quaternionFilters.h"
#include "MPU9250.h"
#include <Wire.h>
#include <stdint.h>


#define SerialDebug false
#define SelfTest false
#define Transmit false
#define Processing true
#define DATA_LENGTH 60

MPU9250 IMU1(NOT_SPI, 1);
MPU9250 IMU2(NOT_SPI, 2);
float time_now, delt_t;
int16_t Mag1_temp[3], Mag2_temp[3];
int16_t count, movement;
float transmit_data[19];
float measure_start;
float input_time[DATA_LENGTH], input_gy2[DATA_LENGTH], input_gy1[DATA_LENGTH], 
      input_ax1[DATA_LENGTH], input_ax2[DATA_LENGTH], input_gz1[DATA_LENGTH] ;
//double* features;
float features[23];

void setup() {
  
  Wire.begin();
  Serial.begin(38400);

  byte c1 = IMU1.readByte(IMU1.getAddr(), WHO_AM_I_MPU9250);
  byte c2 = IMU2.readByte(IMU2.getAddr(), WHO_AM_I_MPU9250);
  
  if (c1 == 0x71)
    Serial.println(F("MPU1 is connected!"));
  else
    Serial.println(F("MPU1 is not connected!"));
  
  if (c2 == 0x71)
    Serial.println(F("MPU2 is connected!"));
  else
    Serial.println(F("MPU2 is not connected!"));

  if (c1 == 0x71 && c2 == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("Both MPU are online..."));

    if(SelfTest)
    {
      // Start by performing self test and reporting values
      IMU1.MPU9250SelfTest(IMU1.selfTest);
      Serial.println(F("IMU1 self reporting:"));
      Serial.print(F("x-axis self test: acceleration trim within : "));
      Serial.print(IMU1.selfTest[0],1); Serial.println("% of factory value");
      Serial.print(F("y-axis self test: acceleration trim within : "));
      Serial.print(IMU1.selfTest[1],1); Serial.println("% of factory value");
      Serial.print(F("z-axis self test: acceleration trim within : "));
      Serial.print(IMU1.selfTest[2],1); Serial.println("% of factory value");
      Serial.print(F("x-axis self test: gyration trim within : "));
      Serial.print(IMU1.selfTest[3],1); Serial.println("% of factory value");
      Serial.print(F("y-axis self test: gyration trim within : "));
      Serial.print(IMU1.selfTest[4],1); Serial.println("% of factory value");
      Serial.print(F("z-axis self test: gyration trim within : "));
      Serial.print(IMU1.selfTest[5],1); Serial.println("% of factory value");
      delay(500);
  
      IMU2.MPU9250SelfTest(IMU2.selfTest);
      Serial.println(F("\n\nIMU2 self reporting:"));
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
    }

    // Calibrate gyro and accelerometers, load biases in bias registers
    IMU1.calibrateMPU9250(IMU1.gyroBias, IMU1.accelBias);
    delay(500);
    IMU2.calibrateMPU9250(IMU2.gyroBias, IMU2.accelBias);
    
    if (SerialDebug)
    {
      Serial.println(F("\nIMU1 Bias values:"));
      Serial.println("Accelerometer1 biases (g)");
      Serial.println(IMU1.accelBias[0]);
      Serial.println(IMU1.accelBias[1]);
      Serial.println(IMU1.accelBias[2]);
  
      Serial.println("Gyro1 biases (deg/s)");
      Serial.println(IMU1.gyroBias[0]);
      Serial.println(IMU1.gyroBias[1]);
      Serial.println(IMU1.gyroBias[2]);    
  
      Serial.println(F("\nIMU2 Bias values:"));
      Serial.println("Accelerometer1 biases (g)");
      Serial.println(IMU2.accelBias[0]);
      Serial.println(IMU2.accelBias[1]);
      Serial.println(IMU2.accelBias[2]);
  
      Serial.println("Gyro1 biases (deg/s)");
      Serial.println(IMU2.gyroBias[0]);
      Serial.println(IMU2.gyroBias[1]);
      Serial.println(IMU2.gyroBias[2]);
    }
    else
    {
      Serial.println("Calibration Compelete");  
    }

    IMU1.initMPU9250();
    delay(500);
    IMU2.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println(F("Both MPU initialized for active data mode...."));
    
    // Get sensor resolutions, only need to do this once
    IMU1.getAres();
    IMU1.getGres();
    
    IMU2.getAres();
    IMU2.getGres();

    
  } // if (both c == 0x71)
  else if(c2 == 0x71)
  {
    Serial.print("Could not connect to MPU1: 0x");
    Serial.println(c1, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
  else if(c1 == 0x71)
  {
    Serial.print("Could not connect to MPU2: 0x");
    Serial.println(c2, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
  else
  {
    Serial.print("Could not connect to MPU1: 0x");
    Serial.println(c1, HEX);
    Serial.print("Could not connect to MPU2: 0x");
    Serial.println(c2, HEX);
    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }

  measure_start = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
    // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
   time_now = millis();
  
  if (IMU1.readByte(IMU1.getAddr(), INT_STATUS) & 0x01)
  {
    IMU1.readAccelData(IMU1.accelCount);  // Read the x/y/z adc values
    
    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    IMU1.ax = (float)IMU1.accelCount[0] * IMU1.aRes;// - IMU1.accelBias[0];
    IMU1.ay = (float)IMU1.accelCount[1] * IMU1.aRes;// - IMU1.accelBias[1];
    IMU1.az = (float)IMU1.accelCount[2] * IMU1.aRes;// - IMU1.accelBias[2];


    IMU1.readGyroData(IMU1.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    IMU1.gx = (float)IMU1.gyroCount[0] * IMU1.gRes;// - IMU1.gyroBias[0];
    IMU1.gy = (float)IMU1.gyroCount[1] * IMU1.gRes;// - IMU1.gyroBias[1];
    IMU1.gz = (float)IMU1.gyroCount[2] * IMU1.gRes;// - IMU1.gyroBias[2];

  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  if (IMU2.readByte(IMU2.getAddr(), INT_STATUS) & 0x01)
  {
    IMU2.readAccelData(IMU2.accelCount);  // Read the x/y/z adc values

    
    IMU2.ax = (float)IMU2.accelCount[0] * IMU2.aRes;// - IMU1.accelBias[0];
    IMU2.ay = (float)IMU2.accelCount[1] * IMU2.aRes;// - IMU1.accelBias[1];
    IMU2.az = (float)IMU2.accelCount[2] * IMU2.aRes;// - IMU1.accelBias[2];

    IMU2.readGyroData(IMU2.gyroCount);  // Read the x/y/z adc values

    
    IMU2.gx = (float)IMU2.gyroCount[0] * IMU2.gRes;// - IMU1.gyroBias[0];
    IMU2.gy = (float)IMU2.gyroCount[1] * IMU2.gRes;// - IMU1.gyroBias[1];
    IMU2.gz = (float)IMU2.gyroCount[2] * IMU2.gRes;// - IMU1.gyroBias[2];
    
  }

    if(SerialDebug)
    {
      Serial.println("\nIMU1 measurements:");
      Serial.print("ax = ");  Serial.print((int)1000 * IMU1.ax);
      Serial.print(" ay = "); Serial.print((int)1000 * IMU1.ay);
      Serial.print(" az = "); Serial.print((int)1000 * IMU1.az);
      Serial.println(" mg");

      Serial.print("gx = ");  Serial.print(IMU1.gx, 2);
      Serial.print(" gy = "); Serial.print(IMU1.gy, 2);
      Serial.print(" gz = "); Serial.print(IMU1.gz, 2);
      Serial.println(" deg/s");

      Serial.print("mx = ");  Serial.print((int)IMU1.mx);
      Serial.print(" my = "); Serial.print((int)IMU1.my);
      Serial.print(" mz = "); Serial.print((int)IMU1.mz);
      Serial.println(" mG");

      Serial.print("q0 = ");  Serial.print(IMU1.q[0]);
      Serial.print(" qx = "); Serial.print(IMU1.q[1]);
      Serial.print(" qy = "); Serial.print(IMU1.q[2]);
      Serial.print(" qz = "); Serial.println(IMU1.q[3]);


      Serial.println("\nIMU2 measurements:");
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

    if(SerialDebug)
    {
      Serial.print("\n IMU1 Yaw, Pitch, Roll: ");
      Serial.print(IMU1.yaw, 2);
      Serial.print(", ");
      Serial.print(IMU1.pitch, 2);
      Serial.print(", ");
      Serial.println(IMU1.roll, 2);

      Serial.print("rate = ");
      Serial.print((float)IMU1.sumCount / IMU1.sum, 2);
      Serial.println(" Hz");

      Serial.print("\n IMU2 Yaw, Pitch, Roll: ");
      Serial.print(IMU2.yaw, 2);
      Serial.print(", ");
      Serial.print(IMU2.pitch, 2);
      Serial.print(", ");
      Serial.println(IMU2.roll, 2);

      Serial.print("rate = ");
      Serial.print((float)IMU2.sumCount / IMU2.sum, 2);
      Serial.println(" Hz");

     IMU1.count = millis();
     IMU1.sumCount = 0;
     IMU1.sum = 0;
     IMU2.sumCount = 0;
     IMU2.sum = 0;
    }


    if(Transmit)
    {
        transmit_data[0] = (millis() - measure_start)/1000.0;
        transmit_data[1] = IMU1.ax; transmit_data[2] = IMU1.ay;transmit_data[3] = IMU1.az;
        transmit_data[4] = IMU1.gx; transmit_data[5] = IMU1.gy;transmit_data[6] = IMU1.gz;
        transmit_data[7] = IMU1.roll; transmit_data[8] = IMU1.pitch;transmit_data[9] = IMU1.yaw;
        transmit_data[10] = IMU2.ax; transmit_data[11] = IMU2.ay;transmit_data[12] = IMU2.az;
        transmit_data[13] = IMU2.gx; transmit_data[14] = IMU2.gy;transmit_data[15] = IMU2.gz;
        transmit_data[16] = IMU2.roll; transmit_data[17] = IMU2.pitch;transmit_data[18] = IMU2.yaw;

        Serial.write((uint8_t*)transmit_data, sizeof(transmit_data));
        Serial.write('\n');
    }

    if(Processing)
    {
      if(count < DATA_LENGTH)
      {
          input_time[count] = millis()/1000.0;
          input_gy2[count] = IMU2.gy;
          input_gy1[count] = IMU1.gy;
          input_ax1[count] = IMU1.ax;
          input_ax2[count] = IMU2.ax;
          input_gz1[count] = IMU1.gz;
          count++;  
      }
      else
      {
          for(int i = 0; i<DATA_LENGTH-1; i++ )
          {
            input_time[i] = input_time[i+1];
            input_gy2[i] = input_gy2[i+1];
            input_gy1[i] = input_gy1[i+1];
            input_ax1[i] = input_ax1[i+1];
            input_ax2[i] = input_ax2[i+1];
            input_gz1[i] = input_gz1[i+1];   
          }

          input_time[DATA_LENGTH-1] = millis()/1000.0;
          input_gy2[DATA_LENGTH-1] = IMU2.gy;
          input_gy1[DATA_LENGTH-1] = IMU1.gy;
          input_ax1[DATA_LENGTH-1] = IMU1.ax;
          input_ax2[DATA_LENGTH-1] = IMU2.ax;
          input_gz1[DATA_LENGTH-1] = IMU1.gz;
          //calculate_mean(input_time, 60);
          movement = extract_features(input_time, input_gy2, input_ax2, input_gy1, input_ax1, input_gz1, DATA_LENGTH, features);
          if(movement)
          {
            Serial.println(movement);
            for(int k =0; k<23; k++)
            {
              Serial.print(features[k]);
              Serial.write(' ');
              //Serial.println(k);
            }
            Serial.println("\n");
            //free(features);
          }
          else
          {
            Serial.println("No movement detected!");  
         }
      }
    }
    Serial.print("Takes time: ");
    Serial.println(millis() - time_now);
    delt_t = time_now + 50 - millis();
    if(delt_t>0)
      delay(delt_t);
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
    mag_reg_wr(mpu, AK8963_CNTL, 0x16);

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
  if (rawData[0] & 0x01)
  {
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
  }
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
    //sample_count = 128;
 /* }
  // at 100 Hz ODR, new mag data is available every 10 ms
  if (Mmode == M_100HZ)
  {*/
    sample_count = 1500;
  //} */

  for (ii = 0; ii < sample_count; ii++)
  { 
    time_now = millis();
    readMag(mpu, mag_temp);  // Read the mag data
    if(mag_temp[0] == 0)
      continue;

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
    delt_t = 10 + time_now - millis();
    delay(delt_t); // At 8 Hz ODR, new mag data is available every 125 ms
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



