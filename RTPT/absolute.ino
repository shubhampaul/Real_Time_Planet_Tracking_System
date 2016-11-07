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
//void magcalMPU9250(float * dest1, float * dest2)
// {
//   Serial.println("B");
// uint16_t ii = 0, sample_count = 0;
// int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
// int16_t mag_max[3] = {0x8000, 0x8000, 0x8000}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};
//
// Serial.println("Mag Calibration: Wave device in a figure eight until done!");
//
// sample_count = 128;
// for(ii = 0; ii < sample_count; ii++) {
// readMagData(mag_temp);  // Read the mag data
// for (int jj = 0; jj < 3; jj++) {
//  if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
//  if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
// }
// delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
// }
//
//// Get hard iron correction
// mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
// mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
// mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
//
// dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
// dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];
// dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];
//
//// Get soft iron correction estimate
// mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
// mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
// mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts
//
// float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
// avg_rad /= 3.0;
//
// dest2[0] = avg_rad/((float)mag_scale[0]);
// dest2[1] = avg_rad/((float)mag_scale[1]);
// dest2[2] = avg_rad/((float)mag_scale[2]);
//
// Serial.println("Mag Calibration done!");
// }
