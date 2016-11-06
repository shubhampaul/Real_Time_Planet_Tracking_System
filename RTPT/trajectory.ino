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

