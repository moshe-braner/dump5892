//
// cpr.cpp - Compact Position Reporting decoding
//
// Adapted from dump1090, a Mode S message decoder for RTLSDR devices.
// Original code copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
// Heavily modified for efficiency - Copyright (C) 2024 by Moshe Braner <moshe.braner@gmail.com>
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version, see <http://www.gnu.org/licenses/>.

#include <math.h>
#include <stdio.h>
#include "dump5892.h"

//
//=========================================================================
//
// The NL function uses the precomputed table from 1090-WP-9-14
//
/*
static int cprNLFunction_original(float lat) {
    if (lat < 0) lat = -lat; // Table is symmetric about the equator
    if (lat < 10.47047130) return 59;
    if (lat < 14.82817437) return 58;
    if (lat < 18.18626357) return 57;
    if (lat < 21.02939493) return 56;
    if (lat < 23.54504487) return 55;
    if (lat < 25.82924707) return 54;
    if (lat < 27.93898710) return 53;
    if (lat < 29.91135686) return 52;
    if (lat < 31.77209708) return 51;
    if (lat < 33.53993436) return 50;
    if (lat < 35.22899598) return 49;
    if (lat < 36.85025108) return 48;
    if (lat < 38.41241892) return 47;
    if (lat < 39.92256684) return 46;
    if (lat < 41.38651832) return 45;
    if (lat < 42.80914012) return 44;
    if (lat < 44.19454951) return 43;
    if (lat < 45.54626723) return 42;
    if (lat < 46.86733252) return 41;
    if (lat < 48.16039128) return 40;
    if (lat < 49.42776439) return 39;
    if (lat < 50.67150166) return 38;
    if (lat < 51.89342469) return 37;
    if (lat < 53.09516153) return 36;
    if (lat < 54.27817472) return 35;
    if (lat < 55.44378444) return 34;
    if (lat < 56.59318756) return 33;
    if (lat < 57.72747354) return 32;
    if (lat < 58.84763776) return 31;
    if (lat < 59.95459277) return 30;
    if (lat < 61.04917774) return 29;
    if (lat < 62.13216659) return 28;
    if (lat < 63.20427479) return 27;
    if (lat < 64.26616523) return 26;
    if (lat < 65.31845310) return 25;
    if (lat < 66.36171008) return 24;
    if (lat < 67.39646774) return 23;
    if (lat < 68.42322022) return 22;
    if (lat < 69.44242631) return 21;
    if (lat < 70.45451075) return 20;
    if (lat < 71.45986473) return 19;
    if (lat < 72.45884545) return 18;
    if (lat < 73.45177442) return 17;
    if (lat < 74.43893416) return 16;
    if (lat < 75.42056257) return 15;
    if (lat < 76.39684391) return 14;
    if (lat < 77.36789461) return 13;
    if (lat < 78.33374083) return 12;
    if (lat < 79.29428225) return 11;
    if (lat < 80.24923213) return 10;
    if (lat < 81.19801349) return 9;
    if (lat < 82.13956981) return 8;
    if (lat < 83.07199445) return 7;
    if (lat < 83.99173563) return 6;
    if (lat < 84.89166191) return 5;
    if (lat < 85.75541621) return 4;
    if (lat < 86.53536998) return 3;
    if (lat < 87.00000000) return 2;
    else return 1;
}
*/

static float NLtable[61] = {
 90.0,        // 0
 90.0,        // 1
 87.00000000, // 2
 86.53536998, // 3
 85.75541621, // 4
 84.89166191, // 5
 83.99173563, // 6
 83.07199445, // 7
 82.13956981, // 8
 81.19801349, // 9
 80.24923213, // 10
 79.29428225, // 11
 78.33374083, // 12
 77.36789461, // 13
 76.39684391, // 14
 75.42056257, // 15
 74.43893416, // 16
 73.45177442, // 17
 72.45884545, // 18
 71.45986473, // 19
 70.45451075, // 20
 69.44242631, // 21
 68.42322022, // 22
 67.39646774, // 23
 66.36171008, // 24
 65.31845310, // 25
 64.26616523, // 26
 63.20427479, // 27
 62.13216659, // 28
 61.04917774, // 29
 59.95459277, // 30
 58.84763776, // 31
 57.72747354, // 32
 56.59318756, // 33
 55.44378444, // 34
 54.27817472, // 35
 53.09516153, // 36
 51.89342469, // 37
 50.67150166, // 38
 49.42776439, // 39
 48.16039128, // 40
 46.86733252, // 41
 45.54626723, // 42
 44.19454951, // 43
 42.80914012, // 44
 41.38651832, // 45
 39.92256684, // 46
 38.41241892, // 47
 36.85025108, // 48
 35.22899598, // 49
 33.53993436, // 50
 31.77209708, // 51
 29.91135686, // 52
 27.93898710, // 53
 25.82924707, // 54
 23.54504487, // 55
 21.02939493, // 56
 18.18626357, // 57
 14.82817437, // 58
 10.47047130, // 59
           0  // 60
};

// this function is recursive - bi-section search
static int cprNLFunction_(float lat, int start, int end) {
    // we already know that NL >= start and NL <= end
    if (end <= start+1) {
        if (lat < NLtable[end])
            return end;
        return start;
    }
    int mid = ((start+end) >> 1);
    if (lat < NLtable[mid])
        return cprNLFunction_(lat, mid, end);
    return cprNLFunction_(lat, start, mid-1);
}

static int cprNLFunction(float lat) {
    if (lat < 0) lat = -lat; // Table is symmetric about the equator
    return cprNLFunction_(lat, 1, 59);
}

// store these numbers to avoid having to repeatdely do the divisions 360/NL & NL/360
static float dLonTable[60];
static float dLonInvTable[60];
// - if we are *really* short on memory space, can wait for GNSS fix first,
//   and then only set the tables up for latitudes near us

// this version uses a pre-computed NL
static float cprDlonFunction(int fflag, int NL) {
    if (NL > 1 && fflag)  NL--;
    return dLonTable[NL];
}
static float cprDlonInvFunction(int fflag, int NL) {
    if (NL > 1 && fflag)  NL--;
    return dLonInvTable[NL];
}

//
//=========================================================================
//
// This algorithm comes from:
// 1090-WP29-07-Draft_CPR101 (which also defines decodeCPR() )
//
// Despite what the earlier comment here said, we should *not* be using trunc().
// See Figure 5-5 / 5-6 and note that floor is applied to (0.5 + fRP - fEP), not
// directly to (fRP - fEP). Eq 38 is correct.
//
int decodeCPRrelative()
{
    // convert incoming cprlxx values to the "fractions" (how far into current zone)
    float fractional_lat = mm.cprlat * 7.629394531e-6;  // = 1/131072 = 2^-17
    float fractional_lon = mm.cprlon * 7.629394531e-6;

    float j, m, rlat, rlon;   // will receive decoded position of target

    // Compute the Latitude Index "j", using the odd/even fflag of the incoming message
    j = flrlat[mm.fflag] + floor(0.5 + modlat[mm.fflag] - fractional_lat);
    // latitude is zone border + the fractional part
    rlat = dLat[mm.fflag] * (j + fractional_lat);
    if (rlat >= 270) rlat -= 360;

    // Check to see that answer is reasonable - i.e. no more than 1/2 zone away
    float degsdiff = fabs(rlat - reflat);
    if (degsdiff > dLatHalf) {
      if (degsdiff > 0.5 * dLat[mm.fflag]) {   // more precise test
        ++msg_by_cpr_effort[3];
if(settings->debug)
Serial.printf("cpr fail: lat %.5f cmp w reflat = %.5f  - %.3f of dLat\n",
rlon, reflon, fabs(rlat-reflat)/dLat[mm.fflag]);
        return (-1);                           // Time to give up - Latitude error
      }
    }

    // 'NL' is the number of logitude zones for the target's latitude.
    // NL[] was pre-computed based on reflat (our location), not rlat (target's).
    // Check whether the pre-computed NL is correct - likely, if target is close.
    //   - correct if: (fabs(rlat) < NLtable[NL] && fabs(rlat) >= NLtable[NL+1])
    float dLon2, scaled, flrlon2, modlon2;
    float absrlat = fabs(rlat);         // the coding of rlon depends on rlat!
    bool gt0 = (absrlat >= NLtable[NL[mm.fflag]]);
    bool lt1 = (absrlat < NLtable[NL[mm.fflag]+1]);
    int effort = 0;
    if (gt0 || lt1) {
        // NL is incorrect for the target, need to recompute dLon etc.
        // First check whether the correct NL is the current one +-1,
        // which is likely if the target is close to the ref location.
        // For these adjacent NLs we've also pre-computed things.
        if (lt1 && absrlat >= NLtable[NL[mm.fflag]+2]) {
            //NL2 = NL[mm.fflag] + 1;
            effort = 1;
            dLon2 = dLonPlus[mm.fflag];
            flrlon2 = flrlonPlus[mm.fflag];
            modlon2 = modlonPlus[mm.fflag];
        } else if (gt0 && absrlat < NLtable[NL[mm.fflag]-1]) {
            //NL2 = NL[mm.fflag] - 1;
            effort = 1;
            dLon2 = dLonMinus[mm.fflag];
            flrlon2 = flrlonMinus[mm.fflag];
            modlon2 = modlonMinus[mm.fflag];
        } else {
            // Shift into non-adjacent zone.  This can be < 100 miles away at lat>45.
            // No choice but to do the full NL search and recompute.
            int NL2 = cprNLFunction_(absrlat, 1, 59);     // = cprNLFunction(rlat)
            effort = 2;
            dLon2 = cprDlonFunction(mm.fflag, NL2);
if(settings->debug>1)
Serial.printf("non-adjacent! NL=%d  NL2=%d   dLon2=%.3f\n", NL[mm.fflag], NL2, dLon2);
            scaled = reflon * cprDlonInvFunction(mm.fflag, NL2);
            flrlon2 = floor(scaled);
            modlon2 = scaled - flrlon2;
        }
    } else {     // pre-computed NL is OK
            dLon2 = dLon[mm.fflag];
            flrlon2 = flrlon[mm.fflag];
            modlon2 = modlon[mm.fflag];
    }

    // Compute the Longitude Index "m"
    m = flrlon2 + floor(0.5 + modlon2 - fractional_lon);
    // longitude is zone border + the fractional part (whew!)
    rlon = dLon2 * (m + fractional_lon);
    if (rlon > 180) rlon -= 360;

    // Check to see that answer is reasonable - i.e. no more than 1/2 zone away
    degsdiff = fabs(rlon - reflon);
    if (degsdiff > dLonHalf) {
      if (degsdiff > 0.5 * dLon2) {    // more precise test
        ++msg_by_cpr_effort[3];
if(settings->debug)
Serial.printf("cpr fail: lon %.5f cmp w reflon = %.5f  - %.3f of dLon\n",
rlon, reflon, fabs(rlon-reflon)/dLon2);
        return (-1);                   // Time to give up - Longitude error
      }
    }

    ++msg_by_cpr_effort[effort];
    fo.latitude  = rlat;
    fo.longitude = rlon;
    return (effort);
}


void CPRRelative_precomp()
{
    // do this every minute or several, using own-ship GNSS position for reflat/reflon
    // - but in the dump5892 "app" our position is static

    // pre-compute all that is possible just based on reference lat/lon:
    // (two each: odd and even versions)

    // float dLat[2];     // degrees interval one zone covers - constant
    // float flrlat[2];   // floor(reflat / dLat) - i.e., zone index (integer value)
    // float modlat[2];   // mod(reflat,dLat)/dLat = reflat/dLat - flrlat[] - i.e., fraction
                          // - because mod(x,y) is defined as: x - y * floor(x/y)
    // uint32_t cprlat    // the "fraction" times 2^17 - this is what is transmitted
    // int NL[2];         // number of longitude zones: fewer at higher latitudes
    // float dLon[2];     // size of degrees interval one zone covers
    // float flrlon[2];   // floor(reflon / dLon) - i.e., zone index
    // float modlon[2];   // mod(reflon,dLon)/dLon = reflon/dLon - flrlon[] - i.e., fraction
    // uint32_t cprlon    // the "fraction" times 2^17

    reflat = settings->latitude;
    reflon = settings->longitude;

    for (int k=0; k<2; k++) {  // odd/even
        float invdLat = (k ? 59.0/360.0 : 60.0/360.0);
        // float scaled = reflat / dLat[k];
        float scaled = reflat * invdLat;
        flrlat[k] = floor(scaled);
        modlat[k] = scaled - flrlat[k];
        float lat0 = dLat[k] * flrlat[k];
        ourcprlat[k] = (uint32_t) ((reflat - lat0)/dLat[k] * (float)(1<<17) + 0.5);
        NL[k] = cprNLFunction(reflat);

        // <<< need to compute NL based on target lat which is not known yet -
        // but when target is close NL is the same, so precompute on speculation
        // - see above in decodeCPRrelative() how this is used
        dLon[k] = cprDlonFunction(k, NL[k]);
        scaled = reflon * cprDlonInvFunction(k, NL[k]);
        flrlon[k] = floor(scaled);
        modlon[k] = scaled - flrlon[k];
        ourcprlon[k] = (uint32_t) ((reflon - dLon[k]*flrlon[k])/dLon[k] * (float)(1<<17) + 0.5);

        // pre-compute cpr values for latitudes at both edges of adjacent NL zones
        // - to allow parse() to detect the zone and compute the distance early
        //     - latitude is lower for higher NL
        //     - our lat is < NLtable[NL[k]], and >= NLtable[NL[k]+1]
        // note these out-of-bounds cpr values are signed!
        float edgelat = NLtable[NL[k]-1];
        if (reflat < 0)  edgelat = -edgelat;
        cprMinuslat[k] = (int32_t)((edgelat-lat0) / dLat[k] * (float)(1<<17) + 0.5);
        edgelat = NLtable[NL[k]];
        if (reflat < 0)  edgelat = -edgelat;
        cprNL0lat[k] = (int32_t)((edgelat-lat0) / dLat[k] * (float)(1<<17) + 0.5);
        edgelat = NLtable[NL[k]+1];
        if (reflat < 0)  edgelat = -edgelat;
        cprNL1lat[k] = (int32_t)((edgelat-lat0) / dLat[k] * (float)(1<<17) + 0.5);
        edgelat = NLtable[NL[k]+2];
        if (reflat < 0)  edgelat = -edgelat;
        cprPluslat[k] = (int32_t)((edgelat-lat0) / dLat[k] * (float)(1<<17) + 0.5);

        // pre-compute some other values for adjacent NL zones

        int NL2 = NL[k] + 1;
        dLonPlus[k] = cprDlonFunction(k, NL2);
        dLonHalf = 0.5 * dLonPlus[0];      // for both odd and even, this value is conservative
        scaled = reflon * cprDlonInvFunction(k, NL2);
        flrlonPlus[k] = floor(scaled);
        modlonPlus[k] = scaled - flrlonPlus[k];
        ourcprlonPlus[k] = (uint32_t) ((reflon - dLonPlus[k]*flrlonPlus[k])/dLonPlus[k] * (float)(1<<17) + 0.5);

        NL2 = NL[k] - 1;
        dLonMinus[k] = cprDlonFunction(k, NL2);
        scaled = reflon * cprDlonInvFunction(k, NL2);
        flrlonMinus[k] = floor(scaled);
        modlonMinus[k] = scaled - flrlonMinus[k];
        ourcprlonMinus[k] = (uint32_t) ((reflon - dLonMinus[k]*flrlonMinus[k])/dLonMinus[k] * (float)(1<<17) + 0.5);

#if defined(TESTING)
if(settings->debug) {
        Serial.printf("[%d] dLat           = %f\n", k, dLat[k]);
        Serial.printf("[%d] flrlat         = %f\n", k, flrlat[k]);
        Serial.printf("[%d] modlat         = %f\n", k, modlat[k]);
        Serial.printf("[%d] ourcprlat      = %d\n", k, ourcprlat[k]);
        Serial.printf("[%d] cprMinuslat    = %d\n", k, cprMinuslat[k]);
        Serial.printf("[%d] cprNL0lat      = %d\n", k, cprNL0lat[k]);
        Serial.printf("[%d] cprNL1lat      = %d\n", k, cprNL1lat[k]);
        Serial.printf("[%d] cprPluslat     = %d\n", k, cprPluslat[k]);
        Serial.printf("[%d] dLon           = %f\n", k, dLon[k]);
        Serial.printf("[%d] dLonPlus       = %f\n", k, dLonPlus[k]);
        Serial.printf("[%d] dLonMinus      = %f\n", k, dLonMinus[k]);
        Serial.printf("[%d] flrlon         = %f\n", k, flrlon[k]);
        Serial.printf("[%d] flrlonPlus     = %f\n", k, flrlonPlus[k]);
        Serial.printf("[%d] flrlonMinus    = %f\n", k, flrlonMinus[k]);
        Serial.printf("[%d] modlon         = %f\n", k, modlon[k]);
        Serial.printf("[%d] modlonPlus     = %f\n", k, modlonPlus[k]);
        Serial.printf("[%d] modlonMinus    = %f\n", k, modlonMinus[k]);
        Serial.printf("[%d] ourcprlon      = %d\n", k, ourcprlon[k]);
        Serial.printf("[%d] ourcprlonPlus  = %d\n", k, ourcprlonPlus[k]);
        Serial.printf("[%d] ourcprlonMinus = %d\n", k, ourcprlonMinus[k]);
}
        // test the computation of our cprlat/lon along with the decoding:
        mm.cprlat = ourcprlat[k];
        mm.cprlon = ourcprlon[k];
        mm.fflag = k;
        decodeCPRrelative();
        Serial.printf("[%d] Reference latitude/longitude: %.5f, %.5f\n", k, reflat, reflon);
        Serial.printf("[%d]    -> Test pre-comp & decode: %.5f, %.5f\n", k, fo.latitude, fo.longitude);
#endif
    }
}

void CPRRelative_setup()
{
    // prepare lookup tables
    for (int i=1; i<60; i++) {    // yes we skip [0] which is not used
        dLonTable[i] = 360.0/i;
        dLonInvTable[i] = i/360.0;
    }

    // these do not change
    dLat[0] = 360.0/60.0;
    dLat[1] = 360.0/59.0;
    // for both odd and even, this value is conservative:
    dLatHalf = 0.5 * dLat[0];
    // first-cut range limit (along each axis):
    maxcprdiff = (int32_t)((float)(1<<16) * (float)settings->maxrange / 180.0);
    // a squared scaled version for slant distance
    maxcprdiff_sq = (maxcprdiff >> 4) * (maxcprdiff >> 4);
    

    // compute what does depend on reflat, reflon
    CPRRelative_precomp();
}
