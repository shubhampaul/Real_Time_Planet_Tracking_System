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

double ipart(double xx)
{
  //Serial.println("IPART");
  double sgn;
  if (xx < 0)
  {
    sgn = -1.0;
  }

  else if (xx == 0)
  {
    sgn = 0.0;
  }

  else if (xx > 0)
  {
    sgn = 1.0;
  }
  double ret = sgn * ((int)fabs(xx));

  return ret;
}


double FNdegmin(double xx)
{
  //Serial.println("DEGMIN");
  double a = ipart(xx) ;
  double b = xx - a ;
  double e = ipart(60 * b) ;
  //   deal with carry on minutes
  if ( e >= 60 )
  {
    e = 0 ;
    a = a + 1 ;
  }
  return (a + (e / 100) );
}

double dayno(int dx, int mx, int yx, double fx)
{
//Serial.println("DAY NO");

  //dno=(367 * yx) -  (int)(7*(yx + (int)((mx + 9) / 12)) / 4) + (int)(275 * mx / 9) + dx - 730531.5 + fx;
  dno = 987 + dx + (fx / 24);
  //Serial.print("\ndays:");
  //Serial.println(dno);
  return dno;
}
double frange(double x)
{
  //Serial.println("FRANGE");
  x = x / (2 * pi);
  x = (2 * pi) * (x - ipart(x));
  if (x < 0)
    x = x + (2 * pi);
  return x;
}
double fkep( double m, double ecc)
{
 
    //  Serial.println(m);  Serial.println(ecc);
  //Serial.println("FKEPA");
 // Serial.println(m);  Serial.println(ecc);
   double e = ecc;

//  do
//  { dyo = e - (ecc * sin(e)) - m;
//    e = e - (dyo / (1 - (ecc * cos(e))));
//    delay(1);
////Serial.print("fabs");Serial.print((fabs(dyo)*pow(10, 6)+1),8);Serial.print("              ");Serial.print("pow");Serial.println((pow(10, -6)+1),8);
//  } while (fabs(dyo)>= pow(10, -12));
//      //Serial.println("fkepB");
//   double v = 2 * atan(sqrt((1 + ecc) / (1 - ecc)) * tan(e / 2));
   
 double v = m + (2 * e - 0.25 *pow(e,3) + 5/96 * pow(e,5)) * sin(m) + (1.25 * pow(e,2) - 11/24 * pow(e,4)) * sin(2*m) + (13/12 * pow(e,3) - 43/64 * pow(e,5)) * sin(3*m) + 103/96 * pow(e,4) * sin(4*m) + 1097/960 * pow(e,5) * sin(5*m);

  if (v < 0)
    v = v + (2 * pi);
  return v;
}
double fnatan(double x, double y)
{
  //Serial.println("ATAN");
  double a = atan(y / x);
  if (x < 0)
    a = a + pi;
  if ((y < 0) && (x > 0))
    a = a + (2 * pi);
  return a;
}
void AltAzCalculate(double RA, double Dec, double Lat, double Long, double hrs, double minut, double dy)
{
  //Serial.println("G");
  // Day offset and Local Siderial Time
  dy = dy + 4975.5;

  double LST = (100.46 + 0.985647 * dy + Long + 15 * (hrs + minut / 60) + 360) - (((int)((100.46 + 0.985647 * dy + Long + 15 * (hrs + minut / 60) + 360) / 360)) * 360);

  // Hour Angle
  double HA = (LST - RA + 360) - ((int)((LST - RA + 360) / 360)) * 360 ;

  // HA, DEC, Lat to Alt, AZ
  double x = cos(HA * (pi / 180)) * cos(Dec * (pi / 180));
  double y = sin(HA * (pi / 180)) * cos(Dec * (pi / 180));
  double z = sin(Dec * (pi / 180));

  double xhor = x * cos((90 - Lat) * (pi / 180)) - z * sin((90 - Lat) * (pi / 180));
  double yhor = y;
  double zhor = x * sin((90 - Lat) * (pi / 180)) + z * cos((90 - Lat) * (pi / 180));

  Azimuth = atan2(yhor, xhor) * (180 / pi) + 180;
  Elevation = asin(zhor) * (180 / pi);
}
void earth()
{
//Serial.println("B");
  M[3] = ((n[3] * rads) * d) + (L[3] - p[3]) * rads;
  M[3] = frange(M[3]);
  v[3] = fkep(M[3], e[3]);
  r[3] = a[3] * ((1 - (pow(e[3], 2))) / (1 + (e[3] * cos(v[3]))));
  x[3] = r[3] * cos(v[3] + p[3] * rads);
  y[3] = r[3] * sin(v[3] + p[3] * rads);
  z[3] = 0;
}
void mainCalculations()
{

  dfrac = hh + (mm / 60);
 d = dayno(dd, mu, yy, dfrac);
 //Serial.println("E");
 earth();
 //Serial.println("F");
//Serial.println("E");
 int j;
 for (j = 0; j <=9; j++)
  {
  if (j == 3)
      continue;
      if(j==9);
  // Serial.println("A");
    M[j] = ((n[j] * rads) * d) + (L[j] - p[j]) * rads;
      if(j==9);
   //   Serial.println("B");

   M[j] = frange(M[j]);
     if(j==9);
   // Serial.println("C");
    v[j] = fkep(M[j], e[j]);
      if(j==9);
     // Serial.println("D");

   r[j] = a[j] * ((1 - pow(e[j], 2)) / (1 + (e[j] * cos(v[j]))));
   x[j] = r[j] * (cos(o[j] * rads) * cos(v[j] + p[j] * rads - o[j] * rads) - sin(o[j] * rads) * sin(v[j] + p[j] * rads - o[j] * rads) * cos(i[j] * rads));
   y[j] = r[j] * (sin(o[j] * rads) * cos(v[j] + p[j] * rads - o[j] * rads) + cos(o[j] * rads) * sin(v[j] + p[j] * rads - o[j] * rads) * cos(i[j] * rads));
   z[j] = r[j] * (sin(v[j] + p[j] * rads - o[j] * rads)) * sin(i[j] * rads);
   Xi[j] = x[j] - x[3];
   Yi[j] = y[j] - y[3];
   Zi[j] = z[j];

   Xq[j] = Xi[j];
    Yq[j] = (Yi[j] * cos(ec)) - (Zi[j] * sin(ec));
    Zq[j] = (Yi[j] * sin(ec)) + (Zi[j] * cos(ec));
    ra[j] = fnatan(Xq[j], Yq[j]);
    dec[j] = atan(Zq[j] / sqrt(pow(Xq[j], 2.0) + pow(Yq[j], 2.0)));
  // Serial.println(j);
  }

//Serial.println("H");
  double alpha = FNdegmin((ra[pno] * degs) / 15);
  double delta = FNdegmin(dec[pno] * degs);

//Serial.println("G");


  AltAzCalculate((alpha * 15), delta, Lat, Long, hh, mm, d);
}


