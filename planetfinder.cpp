#include <iostream>
#include <ctime>
#include <math.h>
#include<stdio.h>
#include <stdlib.h>
#include <string>
using namespace std;
double pi = 4*atan(1);
double tpi = 2 * pi ;
double twopi = tpi ;
double degs = 180 / pi ;
double rads = pi / 180 ;
double el [66] ;

double FNday (int yx, int mx, int dx, double hx)
{
    double a=(367 * yx -  floor(7*(yx + floor((mx + 9) / 12)) / 4) +
floor(275 * mx / 9) + dx - 730531.5 + hx/ 24);
//printf("FNday %f",a);
    return a;
}

double FNatn2(double yx, double xx)
{
    double a=atan(yx/xx);
    if(xx<0)
    {
     a = a + pi ;
    }

    if(( yx < 0 ) && ( xx > 0 ) )
    {
     a = a + tpi ;
    }
    printf("FNatn2 %f\n",a);
    return a;
    }

double FNipart(double xx)
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
double ret=sgn*(int)(abs(xx));
printf("FNipart %f",ret);
    return ret;
    }
//
double FNrange (double xx)
{
double b=xx/tpi ;
double a=tpi*(b-FNipart(b));
if(a<0)
a=tpi+a;
printf("FNrange %f",a);
return (a);
}

//
double FNkep(double m, double ecc, int eps)
{
double e = m ;
double delta = 0.05;
do{
    delta=e-ecc*sin(e)-m;
 // printf("delta %f\n",delta);
     e=e-delta/(1-ecc*cos(e));
     //printf("sam1e %f\n",e);
    }while(abs(delta)>=pow(10,(-12)));


printf("ecc %f\n",ecc);
double v = 2*(atan((sqrt((1+ecc)/(1-ecc) ))*tan(e/2))) ;
//printf("vvv %f\n",v);
if( v < 0 )
v = v + tpi;
printf("truean %f \n",v);
return (v);
}
//
double FNdegmin(double xx)
{//   cosmetic function returns angular values as a made updecimal
//   number  - ddd.mm - the digits after the decimal point arethe
//   minutes.
double a = FNipart(xx) ;
double b = xx - a ;
double e = FNipart(60 * b) ;
//   deal with carry on minutes
if( e>= 60 )
    {
e = 0 ;
a = a + 1 ;
}
printf("FNdegmin %f\n" ,a + (e / 100) );
return (a + (e / 100) );
}

int main() {

el [ 1 ] = 7.00507*rads ;
el [ 2 ] = 48.3339 * rads ;
el [ 3 ] = 77.454 * rads ;
el [ 4 ] = 0.3870978;//# ;
el [ 5 ] = 4.092353* rads ;
el [ 6 ] = 0.2056324;//# ;
el [ 7 ] = 314.42369* rads ;
//Venus
el [ 8 ] = 3.39472 * rads ;
el [ 9 ] = 76.6889 * rads ;
el [ 10 ] = 131.761* rads ;
el [ 11 ] = 0.7233238;//# ;
el [ 12 ] = 1.602158 * rads ;
el [ 13 ] = 0.0067933;
el [ 14 ] = 236.94045* rads ;
//Earth
el [ 15 ] = 0.00041*rads ;
el [ 16 ] = 349.2*rads ;
el [ 17 ] = 102.8517*rads ;
el [ 18 ] = 1.00002;//# ;
el [ 19 ] = 0.9855796*rads ;
el [ 20 ] = 0.0166967;
el [ 21 ] = 328.40353*rads ;
//Mars
el [ 22 ] = 1.84992* rads ;
el [ 23 ] = 49.5664*rads ;
el [ 24 ] = 336.0882* rads ;
el [ 25 ] = 1.5236365;//# ;
el [ 26 ] = 0.5240613 * rads ;
el [ 27 ] = 0.0934231;
el [ 28 ] = 262.42784* rads ;
//Jupiter
el [ 29 ] = 1.30463* rads ;
el [ 30 ] = 100.4713* rads ;
el [ 31 ] = 15.6978* rads ;
el [ 32 ] = 5.202597;//# ;
el [ 33 ] = 0.08309618* rads ;
el [ 34 ] = 0.0484646;
el [ 35 ] = 322.55983* rads ;
//Saturn
el [ 36 ] = 2.48524* rads ;
el [ 37 ] = 113.6358* rads ;
el [ 38 ] = 88.863* rads ;
el [ 39 ] = 9.5719;//# ;
el [ 40 ] = 0.03328656* rads ;
el [ 41 ] = 0.0531651;
el [ 42 ] = 20.95759* rads ;
//Uranus
el [ 43 ] = 0.77343* rads ;
el [ 44 ] = 74.0954* rads ;
el [ 45 ] = 175.6807 * rads ;
el [ 46 ] = 19.30181;//# ;
el [ 47 ] = 0.01162295 * rads ;
el [ 48 ] = 0.0428959;
el [ 49 ] = 303.18967 * rads ;
//Neptune
el [ 50 ] = 1.7681 * rads ;
el [ 51 ] = 131.7925 * rads ;
el [ 52 ] = 7.206 * rads ;
el [ 53 ] = 30.26664;//# ;
el [ 54 ] = 0.005919282 * rads ;
el [ 55 ] = 0.0102981;//# ;
el [ 56 ] = 299.8641 * rads ;
//Pluto
el [ 57 ] = 17.12137 * rads ;
el [ 58 ] = 110.3833 * rads ;
el [ 59 ] = 224.8025 * rads ;
el [ 60 ] = 39.5804;//# ;
el [ 61 ] = 0.003958072 * rads ;
el [ 62 ] = 0.2501272;//# ;
el [ 63 ] = 235.7656 * rads ;
//Dates
el [ 64 ] = 2450680.5;//# ; //date of elements
el [ 65 ] = 2451545;//
//pr$ = "\         \#####.##" ;
//pr2$ = "\         \###.######" ;

//    get the date and planet number from the user
//
int y,m,day,mins,p;
double h,d;
printf("Year\n");scanf("%d",&y);
printf("Month\n");scanf("%d",&m);
printf("Day\n");scanf("%d",&day);
printf("Hour\n");scanf("%d",&h);
printf("Minute\n");scanf("%d",&mins);

h = h + mins / 60 ;
printf("Planet");scanf("%d",&p);
d = FNday(y , m , day , h);
printf("days %f\n",d);
//printf("Days");printf(d);
//
//   get the osculating elements from the list
//   using letters instead of the array element
//   makes the program easier to read.
//
int q = 7 * ( p - 1 ) ;
double ip = el [ q + 1 ] ;
double op = el [ q + 2 ] ;
double pp = el [ q + 3 ] ;
double ap = el [ q + 4 ] ;
double np = el [ q + 5 ] ;
double ep = el [ q + 6 ] ;
double lp = el [ q + 7 ] ;
double ie = el [ 15 ] ;
double oe = el [ 16 ] ;
double pe = el [ 17 ] ;
double ae = el [ 18 ] ;
double ne = el [ 19 ] ;
double ee = el [ 20 ] ;
double le = el [ 21 ] ;
double eldate = el [ 64 ] - 2451545 ;
//
//   now find position of Earth in orbit
//
double mm=ne * ( d - eldate)+ le - pe ;
printf("mmmmm %f\n",mm);
double me = FNrange(mm) ;
double ve = FNkep(me,ee,12);
double re = ae * ((1-pow(ee,2))/(1+(ee*cos(ve))));
printf("\n");
printf("me %f\n",me);
printf("ve %f\n",ve);
printf("re %f\n",re);
printf("ve %f\n",ve);
printf("pe %f\n",pe);
double xe=re*cos(ve+pe);
double ye=re*sin(ve+pe);
double ze=0;
printf("     ");printf("heliocentric coordinates of Earth\n");
printf("X :%f\n",xe);
printf("Y :%f\n",ye); //printf(yh) ;
printf("Z :%f\n",ze); //printf(ze) ;
//
//   and position of planet in its orbit
//
double mp = FNrange((np*(d-eldate))+(lp - pp));
double vp = FNkep(mp,ep ,12);
double rp = ap*(1-(ep * ep))/(1 + ep*cos(vp) ) ;
//
//   heliocentric rectangular coordinates of planet
//
double xh = rp * ( cos( op ) * cos( vp + pp - op ) - sin( op ) * sin( vp + pp - op ) * cos( ip ) ) ;
double yh = rp * ( sin( op ) * cos( vp + pp - op ) + cos( op ) * sin( vp + pp - op ) * cos( ip ) ) ;
double zh = rp * ( sin( vp + pp - op ) * sin( ip ) ) ;

printf("     ");printf("heliocentric coordinates of Planet\n");
printf("X :%f\n",xh);
printf("Y :%f\n",yh); //printf(yh) ;
printf("Z :%f\n",zh);// printf(zh) ;

//   convert to geocentric rectangular coordinates
//
double xg = xh - xe ;
double yg = yh - ye ;
double zg = zh ;
//
//   rotate around x axis from ecliptic to equatorial coords
//
double ecl = 23.429292*rads;//# * rads# ; //value for J2000.0 frame
double xeq = xg ;
double yeq = yg * cos( ecl ) - zg * sin( ecl ) ;
double zeq = yg * sin( ecl ) + zg * cos( ecl ) ;
//
//   find the RA and DEC from the rectangular equatorial coords
//
double ra = FNatn2(yeq , xeq ) ;
double dec = atan( zeq /sqrt( xeq * xeq + yeq * yeq ) ) ;
double rvec = sqrt( xeq * xeq + yeq * yeq + zeq * zeq ) ;
printf("     ");
printf("Equatorial coordinates of planet\n") ;
double RA=FNdegmin(ra * degs / 15);
//printf("      RA : "); printf(RA) ;
double DEC=FNdegmin(dec * degs);
printf ("The absolute value of RA : %f and DEC: %f\n ", RA, DEC);

//printf("Distance : "); printf(rvec) ;
//*********

return 0;
}



