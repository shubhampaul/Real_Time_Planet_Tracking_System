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
#include <stdio.h>
#include<stdio.h>
#include<conio.h>
#include <math.h>
double i[10] = {0.0, 7.0052, 3.3949, 0.0, 1.8496, 1.3033, 2.4869, 0.7728, 1.7692, 17.1695};
double o[10] = {0.0, 48.493, 76.804, 0.0, 49.668, 100.629, 113.732, 73.989, 131.946, 110.469};
double p[10] = {0.0, 77.669, 131.99, 103.147, 336.322, 14.556, 91.500,  169.602, 6.152, 223.486};
double a[10] = {0.0, 0.387098, 0.723327, 1.0000, 1.523762, 5.20245, 9.52450, 19.1882, 29.9987, 39.2766};
double n[10] = {0.0, 4.09235, 1.60215, 0.985611, 0.523998, 0.083099, 0.033551, 0.011733, 0.006002, 0.004006};
double e[10] = {0.0, 0.205645 , 0.006769, 0.016679, 0.093346, 0.048892, 0.055724, 0.047874, 0.009816, 0.246211};
double L[10] = {0.0, 93.8725, 233.5729, 324.5489, 82.9625, 87.9728, 216.6279, 11.9756, 335.0233, 258.8717};
double M[10],v[10],r[10],x[10],y[10],z[10],X[10],Y[10],Z[10],Xq[10],Yq[10],Zq[10],ra[10],dec[10];
double d,dfrac,dno;
int dd,mm,yy,hh,mu;
double pi=4*atan(1);
double rads= (4*atan(1)/180);
double degs= 180/(4*atan(1));



double ipart(double xx)
{
    double sgn;
    if(xx<0)
    {
       sgn=-1.0;
    }

    else if(xx==0)
    {
     sgn=0.0;
    }

    else if(xx>0)
    {
      sgn=1.0;
    }
double ret=sgn*((int)fabs(xx));

    return ret;
    }


double FNdegmin(double xx)
{
double a = ipart(xx) ;
double b = xx - a ;
double e = ipart(60 * b) ;
//   deal with carry on minutes
if( e>= 60 )
    {
e = 0 ;
a = a + 1 ;
}
return (a + (e / 100) );
}

double dayno(int dx,int mx, int yx, double fx)
{


    dno=(367 * yx) -  (int)(7*(yx + (int)((mx + 9) / 12)) / 4) + (int)(275 * mx / 9) + dx - 730531.5 + fx;
    dno-=4975.5;
        printf("days: %f",dno);
    return dno;
}
double frange(double x)
{
    x=x/(2*pi);
    x=(2*pi)*(x-ipart(x));
    if(x<0)
        x=x+(2*pi);
    return x;
}
double fkep( double m, double ecc)
{

   double e = ecc;

 double v = m + (2 * e - 0.25 *pow(e,3) + 5/96 * pow(e,5)) * sin(m) + (1.25 * pow(e,2) - 11/24 * pow(e,4)) * sin(2*m) + (13/12 * pow(e,3) - 43/64 * pow(e,5)) * sin(3*m) + 103/96 * pow(e,4) * sin(4*m) + 1097/960 * pow(e,5) * sin(5*m);

  if (v < 0)
    v = v + (2 * pi);
  return v;
}
double fnatan(double x,double y)
{
    double a=atan(y/x);
    if(x<0)
        a=a+pi;
    if((y<0)&&(x>0))
        a=a+(2*pi);
    return a;
}


 void AltAzCalculate(double RA, double Dec, double Lat, double Long, double hour,double minute,double day)
        {
            // Day offset and Local Siderial Time
day+=4975.5;
            double LST = (100.46 + 0.985647 * day + Long + 15 * (hour + minute / 60) + 360) - (((int)((100.46 + 0.985647 * day + Long + 15 * (hour + minute / 60) + 360)/360))*360);

            // Hour Angle
            double HA = (LST - RA + 360)- ((int)((LST - RA + 360)/360))*360 ;

            // HA, DEC, Lat to Alt, AZ
            double x = cos(HA * (pi / 180)) * cos(Dec * (pi / 180));
            double y = sin(HA * (pi / 180)) *cos(Dec * (pi / 180));
            double z = sin(Dec * (pi / 180));

            double xhor = x * cos((90 - Lat) * (pi / 180)) - z *sin((90 - Lat) * (pi / 180));
            double yhor = y;
            double zhor = x * sin((90 - Lat) * (pi / 180)) + z *cos((90 - Lat) * (pi / 180));

            double az = atan2(yhor, xhor) * (180 / pi) + 180;
            double alt = asin(zhor) * (180 / pi);
             printf("\n newAzimuth: %f \n",az);
    printf("\n newElevation: %f \n",alt);


        }

void earth()
{

M[3]=((n[3]*rads)*(d))+(L[3]-p[3])*rads;
M[3]=frange(M[3]);
v[3]=fkep(M[3],e[3]);
r[3]=a[3]*((1 - (pow(e[3],2)))/(1+ (e[3]*cos(v[3]))));
x[3]=r[3]*cos(v[3]+p[3]*rads);
y[3]=r[3]*sin(v[3]+p[3]*rads);
z[3]=0;
printf("Heliocentric coordinates of earth: \n");
printf("%f\n%f\n%f\n",x[3],y[3],z[3]);
}
int main()
{
   printf("Enter year");
    scanf("%d",&yy);
    printf("Enter month");
scanf("%d",&mu)   ;
    printf("Enter day");
    scanf("%d",&dd);


   printf("Enter Hours");
 scanf("%d",&hh);
    printf("Enter Minutes");
   scanf("%d",&mm);

    dfrac=(hh+(mm/60))/24;
    d=dayno(dd,mu,yy,dfrac);

 earth();
int j;
for(j=0;j<10;j++)
{
    if(j==3)
        continue;
M[j]=(n[j]*rads*(d))+(L[j]-p[j])*rads;

M[j]=frange(M[j]);
v[j]=fkep(M[j],e[j]);

r[j]=a[j]*((1 - pow(e[j],2))/(1+ (e[j]*cos(v[j]))));
x[j]=r[j] * (cos(o[j]*rads) * cos(v[j] + p[j]*rads - o[j]*rads) - sin(o[j]*rads) * sin(v[j] + p[j]*rads - o[j]*rads) * cos(i[j]*rads));
y[j]=r[j] * (sin(o[j]*rads) * cos(v[j] + p[j]*rads - o[j]*rads) + cos(o[j]*rads) * sin(v[j] + p[j]*rads - o[j]*rads) * cos(i[j]*rads));
z[j] = r[j]*(sin(v[j] + p[j]*rads - o[j]*rads)) * sin(i[j]*rads);
X[j]=x[j]-x[3];
Y[j]=y[j]-y[3];
Z[j]=z[j];

double ec=23.439292*rads;
Xq[j]=X[j];
Yq[j]=(Y[j]*cos(ec))-(Z[j]*sin(ec));
Zq[j]=(Y[j]*sin(ec))+(Z[j]*cos(ec));
ra[j]=fnatan(Xq[j],Yq[j]);
dec[j]=atan(Zq[j]/sqrt(pow(Xq[j],2.0)+pow(Yq[j],2.0)));
}

int pno;
printf("Enter planet no");
scanf("%d",&pno);
printf("Heliocentric coordinates of planet: \n");
printf("%f\n%f\n%f\n ",x[pno],y[pno],z[pno]);
printf("equitorial coordinates of planet: \n");
printf("%f\n%f\n%f\n ",Xq[pno],Yq[pno],Zq[pno]);
printf("paraeters\n");
printf("mean anomaly : %f\n true anomaly : %f\n radius vector : %f\n ",M[pno],v[pno],r[pno]);
double alpha= FNdegmin((ra[pno]*degs)/15);
double delta=FNdegmin(dec[pno]*degs);
printf("Right ascension: %f",alpha);
printf("\n");
printf("Declination: %f",delta);
AltAzCalculate(((alpha)*15), (delta),12.824278,80.042885, hh, mm,d);
//main();

return 0;
}

