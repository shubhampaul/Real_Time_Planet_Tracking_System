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
 */
int trajec()
{
  hh=0;
  mm=0;
 while(hh<24&& (digitalRead(46) == 0))
 {
   while(mm<60&& (digitalRead(46) == 0))
   {
     
mainCalculations();


nyaw = 360 - yaw;

  Azim = Azimuth - nyaw;
  Azim -= 90;
  while (Azim < 0)
    Azim = 360.0 - abs(Azim);

  Azi = map(Azim, 0, 360, 5, 29);
  Az = (int)Azi;
  Elev = map(Elevation, -90, 90, 2, 178);
  El = (int)Elev;
  myservoAz.write(Az);
  myservoEl.write(El);
  mm++;
  Serial.print(hh); Serial.print(":"); Serial.println(mm);
   }
   mm=0;
   hh++;
   while(hh==19)
   hh=20;
   Serial.println("good");
 }
 return 5;
}

