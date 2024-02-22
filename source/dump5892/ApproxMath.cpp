/*
 * ApproxMath.cpp
 * Copyright (C) 2022,2024 Moshe Braner
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "dump5892.h"
#include "ApproxMath.h"

/* For the purposes used here, trig functions don't need much precision */
/* - on the other hand can save CPU time by using faster approximations */

/* helper function, only valid for positive arguments      */
/* quadratic fit on 0-45 range, results within +-0.25 deg  */
static float atan_positive(float ns, float ew)
{
  float t;
  if (ew < ns) {
    t = ew / ns;
    return (45.0*t + 15.5*(t*(1.0-t)));
  } else {
    t = ns / ew;
    return (90.0 - 45.0*t - 15.5*(t*(1.0-t)));
  }
}

/* An approximation to atan2()                         */
/* - returned value is in degrees clockwise-from-North */
/* - arguments are in reverse order from library atan2 */
float atan2_approx(float ns, float ew)
{
  if (ew > 0.0) {
    if (ns > 0.0) return atan_positive(ns,ew);
    if (ns < 0.0) return 180.0 - atan_positive(-ns,ew);
    /* if (ns==0) */ return 90.0;
  } else if (ew < 0.0) {
    if (ns > 0.0) return 360.0 - atan_positive(ns,-ew);
    if (ns < 0.0) return 180.0 + atan_positive(-ns,-ew);
    /* if (ns==0) */ return 270.0;
  } else {  /* if (ew==0) */
    if (ns >= 0.0) return 0.0;
    return 180.0;
  }
}

// even faster integer version (but cannot avoid one integer division)
// - accurate to within about 1 degree

static int32_t iatan_positive(int32_t ns, int32_t ew)
{
    if (ew > ns)
        return (90 - iatan_positive(ew, ns));
    uint32_t t;
    if (ew < 0x000FFFFF) {
      t = (((uint32_t)ew) << 8);
      t /= (uint32_t)ns;              // 0..256 range since ew <= ns
    } else {
      t = (((uint32_t)ns) >> 8);
      t = (uint32_t)ew / t;
    }
    return (int32_t)((45*t + ((t*(256-t))>>4) + 128)>>8);
}

int32_t iatan2_approx(int32_t ns, int32_t ew)
{
  if (ew > 0) {
    if (ns > 0) return iatan_positive(ns,ew);
    if (ns < 0) return 180 - iatan_positive(-ns,ew);
    /* if (ns==0) */ return 90;
  } else if (ew < 0) {
    if (ns > 0) return 360 - iatan_positive(ns,-ew);
    if (ns < 0) return 180 + iatan_positive(-ns,-ew);
    /* if (ns==0) */ return 270;
  } else {  /* if (ew==0) */
    if (ns >= 0) return 0;
    return 180;
  }
}

/* Bhaskara's (7th century) approximation for the Sine and Cosine, expanded range: */
/*   https://scholarworks.umt.edu/cgi/viewcontent.cgi?article=1313&context=tme     */

/* approximate sin(), argument in degrees, meant for +-360deg range */
float sin_approx(float degs)
{
  bool neg;
  float prathama, sine;

  neg = (degs < 0.0);
  if (neg)  degs = -degs;
  if (degs > 360.0)  degs = degs - 360.0;
  if (degs > 360.0)  degs = degs - 360.0;
  if (degs > 180.0) {
    prathama = (degs - 180.0) * (360.0 - degs);
    neg = !neg;
  } else {
    prathama = degs * (180.0 - degs);
  }
  sine = 4.0*prathama / (40500.0-prathama);
  if (neg)  return -sine;
  return sine;
}

float cos_approx(float degs)
{
  return sin_approx(degs+90.0);
}

/*
 * Approximate sqrt(x^2+y^2):
 * 
 * Based on: https://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
 * with one added "iteration" (at a cost of a float division).
 * 
 * Maximum error < 0.07%, average error about 0.03%.
 */
float approxHypotenuse(float x, float y)
{
   x = fabs(x);
   y = fabs(y);
   if (x == 0.0) {
     return y;
   } else if (y == 0.0) {
     return x;
   } else {
     float h;
     if (x < y)  { h=x;  x=y;  y=h; }
     if ( x < 16.0 * y ) {
         h = ( x * (0.983398 - 0.0390625)) + ( y * 0.430664 );
     } else {
         h = ( x * 0.983398 ) + ( y * 0.430664 );
     }
     return (0.5 * ((x*x + y*y) / h + h));
   }
}

// even faster integer version (the original code):
// - accuracy about +- 4%
uint32_t iapproxHypotenuse( uint32_t dx, uint32_t dy )
{
   uint32_t imin, imax, approx;
   if ( dx < 0 ) dx = -dx;
   if ( dy < 0 ) dy = -dy;
   if ( dx < dy ) {
      imin = dx;
      imax = dy;
   } else {
      imin = dy;
      imax = dx;
   }
   approx = ( imax * 1007 ) + ( imin * 441 );
   if ( imax < ( imin << 4 ))
      approx -= ( imax * 40 );
   // add 512 for proper rounding
   return (( approx + 512 ) >> 10 );
}

/* cos(latitude) is used to convert longitude difference into linear distance. */
/* Once computed, accurate enough through a significant range of latitude. */

static float cos_lat = 0.7071;
static float inv_cos_lat = 1.4142;

float CosLat(float latitude)
{
  static float oldlat = 45.0;
  if (fabs(latitude-oldlat) > 0.3) {
    cos_lat = cos_approx(latitude);
    if (cos_lat > 0.01)
        inv_cos_lat = 1.0 / cos_lat;
    oldlat = latitude;
  }
  return cos_lat;
}

float InvCosLat() { return inv_cos_lat; }
