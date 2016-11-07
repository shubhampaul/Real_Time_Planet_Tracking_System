/*
RTPT (Real Time Planet Tracking System and Trajectory Prediction)
Copyright © 2016  Shubham Paul , Samhita Ganguly ,Rohit Kumar
This file is part of RTPT.
    RTPT is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    RTPT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with RTPT.  If not, see <http://www.gnu.org/licenses/>.
 MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 */
void MPUloop()
{
  //Serial.println("A");
  // If intPin goes high, all data registers have new data
  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0] * aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes; // - accelBias[1];
    az = (float)accelCount[2] * aRes; // - accelBias[2];

    readGyroData(gyroCount);  // Read the x/y/z adc values
    getGres();

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;

    readMagData(magCount);  // Read the x/y/z adc values
    getMres();
    //    magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    //    magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
    //    magbias[2] = +125.;  // User environmental x-axis correction in milliGauss

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
//           mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
//           my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];
//           mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];
    mx = (float)magCount[0] * mRes * magCalibration[0] + 451.95; // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1] * mRes * magCalibration[1] + 550.21;
    mz = (float)magCount[2] * mRes * magCalibration[2] + 451.95;
  }

  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f,  my,  mx, mz);
  //  MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);


  if (!AHRS) {
    delt_t = millis() - count;
    if (delt_t > 500) {

      if (SerialDebug) {
        // Print acceleration values in milligs!
        //    Serial.print("X-acceleration: "); Serial.print(1000*ax); Serial.print(" mg ");
        //    Serial.print("Y-acceleration: "); Serial.print(1000*ay); Serial.print(" mg ");
        //    Serial.print("Z-acceleration: "); Serial.print(1000*az); Serial.println(" mg ");
        //
        //    // Print gyro values in degree/sec
        //    Serial.print("X-gyro rate: "); Serial.print(gx, 3); Serial.print(" degrees/sec ");
        //    Serial.print("Y-gyro rate: "); Serial.print(gy, 3); Serial.print(" degrees/sec ");
        //    Serial.print("Z-gyro rate: "); Serial.print(gz, 3); Serial.println(" degrees/sec");
        //
        //    // Print mag values in degree/sec
        //    Serial.print("X-mag field: "); Serial.print(mx); Serial.print(" mG ");
        //    Serial.print("Y-mag field: "); Serial.print(my); Serial.print(" mG ");
        //    Serial.print("Z-mag field: "); Serial.print(mz); Serial.println(" mG");

        tempCount = readTempData();  // Read the adc values
        temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
        // Print temperature in degrees Centigrade
        Serial.print("Temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
      }

      //    display.clearDisplay();
      //    display.setCursor(0, 0); Serial.println("MPU9250/AK8963");
      //    display.setCursor(0, 8); Serial.println(" x   y   z  ");
      //
      //    display.setCursor(0,  16); Serial.println((int)(1000*ax));
      //    display.setCursor(24, 16); Serial.println((int)(1000*ay));
      //    display.setCursor(48, 16); Serial.println((int)(1000*az));
      //    display.setCursor(72, 16); Serial.println("mg");
      //
      //    display.setCursor(0,  24); Serial.println((int)(gx));
      //    display.setCursor(24, 24); Serial.println((int)(gy));
      //    display.setCursor(48, 24); Serial.println((int)(gz));
      //    display.setCursor(66, 24); Serial.println("o/s");
      //
      //    display.setCursor(0,  32); Serial.println((int)(mx));
      //    display.setCursor(24, 32); Serial.println((int)(my));
      //    display.setCursor(48, 32); Serial.println((int)(mz));
      //    display.setCursor(72, 32); Serial.println("mG");
      //
      //    display.setCursor(0,  40); Serial.println("Gyro T ");
      //    display.setCursor(50,  40); Serial.println(temperature, 1); Serial.println(" C");
      //    display.display();
      //
      count = millis();
      digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    }
  }
  else {

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 50) { // update LCD once per half-second independent of read rate

      if (SerialDebug) {
        //    Serial.print("ax = "); Serial.print((int)1000*ax);
        //    Serial.print(" ay = "); Serial.print((int)1000*ay);
        //    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
        //    Serial.print("gx = "); Serial.print( gx, 2);
        //    Serial.print(" gy = "); Serial.print( gy, 2);
        //    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
        //    Serial.print("mx = "); Serial.print( (int)mx );
        //    Serial.print(" my = "); Serial.print( (int)my );
        //    Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");
        //
        //    Serial.print("q0 = "); Serial.print(q[0]);
        //    Serial.print(" qx = "); Serial.print(q[1]);
        //    Serial.print(" qy = "); Serial.print(q[2]);
        //    Serial.print(" qz = "); Serial.println(q[3]);
      }

      // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
      // In this coordinate system, the positive z-axis is down toward Earth.
      // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
      // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
      // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
      // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
      // applied in the correct order which for this configuration is yaw, pitch, and then roll.
      // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
      yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
      pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
      roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      pitch *= 180.0f / PI;
      yaw   *= 180.0f / PI;
      yaw   += 1.34; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
      roll  *= 180.0f / PI;

      yaw = yaw + 180;
      //Serial.print("Yaw, Pitch, Roll: ");
      //Serial.print(yaw, 2);


      //        yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
      //    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
      //    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      //    pitch *= 180.0f / PI;
      //    yaw   *= 180.0f / PI;
      //    yaw   = 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
      //    roll  *= 180.0f / PI;

      //    if(SerialDebug) {
      //    Serial.print("Yaw, Pitch, Roll: ");
      //    Serial.print(yaw, 2);
      //    Serial.print(", ");
      //    Serial.print(pitch, 2);
      //    Serial.print(", ");
      //    Serial.println(roll, 2);
      //
      ////  Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
      //    }

      //    display.clearDisplay();
      //
      //    display.setCursor(0, 0); Serial.println(" x   y   z  ");
      //
      //    display.setCursor(0,  8); Serial.println((int)(1000*ax));
      //    display.setCursor(24, 8); Serial.println((int)(1000*ay));
      //    display.setCursor(48, 8); Serial.println((int)(1000*az));
      //    display.setCursor(72, 8); Serial.println("mg");
      //
      //    display.setCursor(0,  16); Serial.println((int)(gx));
      //    display.setCursor(24, 16); Serial.println((int)(gy));
      //    display.setCursor(48, 16); Serial.println((int)(gz));
      //    display.setCursor(66, 16); Serial.println("o/s");
      //
      //    display.setCursor(0,  24); Serial.println((int)(mx));
      //    display.setCursor(24, 24); Serial.println((int)(my));
      //    display.setCursor(48, 24); Serial.println((int)(mz));
      //    display.setCursor(72, 24); Serial.println("mG");
      //
      //    display.setCursor(0,  32); Serial.println((int)(yaw));
      //    display.setCursor(24, 32); Serial.println((int)(pitch));
      //    display.setCursor(48, 32); Serial.println((int)(roll));
      //    display.setCursor(66, 32); Serial.println("ypr");
      //
      // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and
      // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
      // The filter update rate is determined mostly by the mathematical steps in the respective algorithms,
      // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
      // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
      // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively.
      // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
      // This filter update rate should be fast enough to maintain accurate platform orientation for
      // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
      // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
      // The 3.3 V 8 MHz Pro Mini is doing pretty well!
      //    display.setCursor(0, 40);
      //    Serial.println("rt: "); Serial.println((float) sumCount / sum, 2); Serial.println(" Hz");
      //    display.display();

      count = millis();
      sumCount = 0;
      sum = 0;
    }
  }

}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void getMres() {
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 10.*4912. / 8190.; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      mRes = 10.*4912. / 32760.0; // Proper scale to return milliGauss
      break;
  }
}

void getGres() {
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
      gRes = 250.0 / 32768.0;
      break;
    case GFS_500DPS:
      gRes = 500.0 / 32768.0;
      break;
    case GFS_1000DPS:
      gRes = 1000.0 / 32768.0;
      break;
    case GFS_2000DPS:
      gRes = 2000.0 / 32768.0;
      break;
  }
}

void getAres() {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
      aRes = 2.0 / 32768.0;
      break;
    case AFS_4G:
      aRes = 4.0 / 32768.0;
      break;
    case AFS_8G:
      aRes = 8.0 / 32768.0;
      break;
    case AFS_16G:
      aRes = 16.0 / 32768.0;
      break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
    }
  }
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128) / 256. + 1.;
  destination[2] =  (float)(rawData[2] - 128) / 256. + 1.;
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}


void initMPU9250()
{
  // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

  // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG);
  //  writeRegister(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x02); // Clear Fchoice bits [1:0]
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
  // writeRegister(GYRO_CONFIG, c | 0x00); // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG

  // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
  //  writeRegister(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  delay(100);
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

  }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
  }
  else {
    accel_bias[2] += (int32_t) accelsensitivity;
  }

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4)       & 0xFF;
  data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4)       & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

  // Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for (ii = 0; ii < 3; ii++) {
    if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

  // Output scaled accelerometer biases for display in the main program
  dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
  dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
  dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;


}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
  float factoryTrim[6];
  uint8_t FS = 0;

  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1 << FS); // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1 << FS); // Set full scale range for the accelerometer to 2 g

  for ( int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

  // Configure the accelerometer for self-test
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(25);  // Delay a while to let the device stabilize

  for ( int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
  delay(25);  // Delay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
  selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
  selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
  selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
  selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
  selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++) {
    destination[i]   = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i]; // Report percent differences
    destination[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3]; // Report percent differences
  }

}


// Wire.h read and write protocols
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }         // Put read results in the Rx buffer
}

