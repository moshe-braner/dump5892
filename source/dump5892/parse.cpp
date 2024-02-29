/*
 * parse.cpp
 * Copyright (C) 2024 Moshe Braner
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version - see <http://www.gnu.org/licenses/>.
 */

#include "dump5892.h"
#include "ApproxMath.h"

// This file based in part on https://github.com/watson/libmodes/

// we have the 112-bit (14 byte) message as hex digits, 2 per byte
// (28 chars) preceded with '*' and finished by ';' + <CR><LF>
// for example: *8D4B1621994420C18804887668F9;
// (the shorter mode-S messages have fewer (14) chars,
// for example: *02E198BFAF8676;
// Or, in mode 2+ with an RSSI prefix:
// starts with '+', 2 chars RSSI, rest as above:
// for example:  +1A8DC03ABC9901939CA00706079C17;

// The transponder ES messages (DF17) are structured as:
// DF: 5 bits
// CA: 3 bits
// ICAO ID: 24 bits
// ME (message body): 56 bits
// PI (CRC etc): 24 bits - can ignore?

#define byte2hex_msb(b) hex[((b)>>4)&0x0F]
#define byte2hex_lsb(b) hex[(b)&0x0F]

// assume the data is always valid and uppercase
//int hex2bin(char c) {
//    if (c >= 'A')  return (0xA + (c - 'A'));
//    return (c - '0');
//}
// make this inline:
#define hex2bin(c) (((c)>='A')? (0xA+((c)-'A')) : ((c)-'0'))

// Decode the 12 bit AC altitude field (in DF 17 and others). Returns the
// altitude or 0 if it can't be decoded.
static int decode_ac12_field() {
    if ((msg[5] | ((msg[6]&0xF0) << 4)) == 0) {
        ++msg_by_alt_cat[0];
        return 0;             // all bits 0 = altitude unknown
    }
    if ((msg[5] & 1) == 0) {    // q_bit not set
        ++msg_by_alt_cat[3];
        return 99999;    // high altitude, decoding method unknown
    }
    // N is the 11 bit integer resulting from the removal of bit Q
    int n = ((msg[5]>>1)<<4) | ((msg[6]&0xF0) >> 4);
    // The final altitude is due to the resulting number multiplied by 25,
    // minus 1000.
    n = n*25-1000;
    if (n < 18000)
        ++msg_by_alt_cat[1];
    else
        ++msg_by_alt_cat[2];
    return n;
}

static bool parse_identity(bool justparse, char s)
{
    int k = parsedchars;

    // raw_aircraft_type = msg[4];  // combines mm.type & mm.sub
    if (mm.sub == 0)
        fo.aircraft_type = 0;
    else if (mm.type == 2)
        fo.aircraft_type = 0;   // surface vehicle or object
    else fo.aircraft_type = msg[4] - 0x18;
        // 0x0 unknown
        // 0x1 glider
        // 0x2 LTA 
        // 0x3 parachute 
        // 0x4 hang glider
        // 0x5 reserved
        // 0x6 UAV
        // 0x7 spacecraft
        // 0x8 not used
        // 0x9 light 
        // 0xA med1
        // 0xB med2
        // 0xC high vortex
        // 0xD heavy
        // 0xE high perf
        // 0xF rotorcraft

    // filter by aircraft type
    // if (settings->ac_type != 0 && fo.aircraft_type != settings->ac_type)
    //     return false;
    // - instead let update_traffic_position() handle it

    //raw_callsign = last 6 bytes
    if (justparse) {
        for (int m=0; m<6; m++) {
            uint8_t c = msg[5+m];
            parsed[k++] = byte2hex_msb(c);
            parsed[k++] = byte2hex_lsb(c);
        }
    }

    // decode callsign
    static const char *ais_charset = "@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_ !\"#$%&'()*+,-./0123456789:;<=>?";
    // Note that mapping 6-bit binary values into this table always results in a printable character.
    // That is why callsign[0]==0 is a valid check for not having received an identity message.
    fo.callsign[0] = ais_charset[msg[5]>>2];
    fo.callsign[1] = ais_charset[((msg[5]&3)<<4)|(msg[6]>>4)];
    fo.callsign[2] = ais_charset[((msg[6]&15)<<2)|(msg[7]>>6)];
    fo.callsign[3] = ais_charset[msg[7]&63];
    fo.callsign[4] = ais_charset[msg[8]>>2];
    fo.callsign[5] = ais_charset[((msg[8]&3)<<4)|(msg[9]>>4)];
    fo.callsign[6] = ais_charset[((msg[9]&15)<<2)|(msg[10]>>6)];
    fo.callsign[7] = ais_charset[msg[10]&63];
    fo.callsign[8] = '\0';

if(settings->debug>1)
Serial.printf("identity: %06X %s\n", fo.addr, fo.callsign);

    if (! justparse)
        update_traffic_identity();

    parsedchars = k;
    return true;
}


static bool parse_position(bool justparse, char s)
{
    int k = parsedchars;

    mm.fflag = ((msg[6] & 0x4) >> 2);
    //tflag = msg[6] & 0x8;
    mm.cprlat = ((msg[6] & 3) << 15) | (msg[7] << 7) | (msg[8] >> 1);
    mm.cprlon = ((msg[8]&1) << 16) | (msg[9] << 8) | msg[10];

    // Most receiveable signals are from farther away than we may be interested in.
    // An efficient way to filter them out at this early stage will save a lot of CPU cycles.

    int32_t m = (int32_t) mm.cprlat;
    int32_t r = (int32_t) ourcprlat[mm.fflag];   // convert from unsigned to signed...
    if (m-r > (1<<16)) {
        // maybe it's just wraparound on the edge of the zone
        // - choose the closer interpretation
        m -= (1<<17);
if(settings->debug>1)
Serial.println("lat wraparound down...");
    } else if (r-m > (1<<16)) {
        m += (1<<17);
if(settings->debug>1)
Serial.println("lat wraparound up...");
    }
    int32_t cprlatdiff = m - r;
    int32_t abslatdiff = abs(cprlatdiff);
    if (abslatdiff > maxcprdiff) {        // since even just lat diff is too far
      if (fo.addr != settings->follow)
        return false;                     // no need to compute slant distance
    }

    // identify the NL zone, ours, an adjacent one, or beyond
    bool adjacent = true;
    if (reflat < 7.5 && reflat > 7.5) {              // one big NL zone around the equator
        r = (int32_t) ourcprlon[mm.fflag];
    } else if (reflat > 0) {
      if (m < cprNL1lat[mm.fflag]) {                 // target lat in higher-NL zone
        if (m < cprPluslat[mm.fflag]) {              // beyond the adjacent zone
            adjacent = false;
        }
        r = (int32_t) ourcprlonPlus[mm.fflag];
      } else if (m > cprNL0lat[mm.fflag]) {          // target lat in lower-NL zone
        if (m > cprMinuslat[mm.fflag]) {             // beyond the adjacent zone
            adjacent = false;
        }
        r = (int32_t) ourcprlonMinus[mm.fflag];
      } else {
        r = (int32_t) ourcprlon[mm.fflag];
      }
    } else {                                         // reflat < 0
      if (m > cprNL1lat[mm.fflag]) {                 // in higher-NL zone (towards equator)
        if (m > cprPluslat[mm.fflag]) {              // beyond the adjacent zone
            adjacent = false;
        }
        r = (int32_t) ourcprlonPlus[mm.fflag];
      } else if (m < cprNL0lat[mm.fflag]) {          // in lower-NL zone (towards south pole)
        if (m < cprMinuslat[mm.fflag]) {             // beyond the adjacent zone
            adjacent = false;
        }
        r = (int32_t) ourcprlonMinus[mm.fflag];
      } else {
        r = (int32_t) ourcprlon[mm.fflag];
      }
    }
    m = (int32_t) mm.cprlon;
    if (m-r > (1<<16)) {
        m -= (1<<17);
if(settings->debug>1)
Serial.println("lon wraparound down...");
    } else if (r-m > (1<<16)) {
        m += (1<<17);
if(settings->debug>1)
Serial.println("lon wraparound up...");
    }
    int32_t cprlondiff = m - r;
    int32_t abslondiff = abs(cprlondiff);
    if (adjacent) {
      if (fo.addr != settings->follow) {
        if (abslondiff > maxcprdiff)
            return false;
        // weed out remaining too-far using pre-computed squared-hypotenuse
        // - no need to use hypotenus-approximation
        if ((abslondiff>>4) * (abslatdiff>>4) > maxcprdiff_sq)
            return false;
      }
    } else {
if (settings->debug)
Serial.printf("position: non-adjacent, distance unknown\n");
    }

    // altitude is in msg[5] & MSnibble of msg[6]
    if (mm.type <= 18) {     // baro alt
        fo.alt_type = 0;
        fo.altitude = decode_ac12_field();
    } else {      // GNSS alt, rare
        fo.alt_type = 1;
        mm.msgtype = 'G';
        fo.altitude = (msg[5] << 4) | ((msg[6] >> 4) & 0x0F);   // meters!
        fo.altitude = ((fo.altitude * 3360) >> 10);
        ++msg_by_alt_cat[fo.altitude < 18000? 1 : 2];
    }

if(settings->debug>1)
Serial.printf("position: altitude: %d\n", fo.altitude);

    // filter by altitude, but always include "followed" aircraft
    if (fo.addr != settings->follow) {
        int alts = settings->alts;
        if (alts == LOWALT && fo.altitude > 18000) {
            //--msg_by_dst_cat[far];    // try and undo the increment earlier
            //--msg_by_alt_cat[2];
            return false;
        }
        if (alts == MEDALT && fo.altitude < 18000) {
            //--msg_by_dst_cat[far];
            //--msg_by_alt_cat[1];
            return false;
        }
        if (alts == HIGHALT && fo.altitude < 50000) {
            //--msg_by_dst_cat[far];
            //--msg_by_alt_cat[fo.altitude < 18000? 1 : 2];
            return false;
        }
    }

    if (justparse) {
        //int alt = (msg[5] << 4) | ((msg[6] >> 4) & 0x0F);   // 12 bits
        // altitude - 12 bits
        parsed[k++] = byte2hex_msb(msg[5]);
        parsed[k++] = byte2hex_lsb(msg[5]);
        parsed[k++] = byte2hex_msb(msg[6]);
        parsed[k++] = s;
        parsed[k++] = hex[(msg[6] & 0x8) >> 3];    // tflag
        parsed[k++] = s;
        parsed[k++] = hex[mm.fflag];
        parsed[k++] = s;
        // cpr lat/lon - 17 bits each
        parsed[k++] = hex[(mm.cprlat & 0x10000) >> 16];
        parsed[k++] = hex[(mm.cprlat & 0x0F000) >> 12];
        parsed[k++] = hex[(mm.cprlat & 0x00F00) >>  8];
        parsed[k++] = hex[(mm.cprlat & 0x000F0) >>  4];
        parsed[k++] = hex[(mm.cprlat & 0x0000F)];
        parsed[k++] = s;
        parsed[k++] = hex[(mm.cprlon & 0x10000) >> 16];
        parsed[k++] = hex[(mm.cprlon & 0x0F000) >> 12];
        parsed[k++] = hex[(mm.cprlon & 0x00F00) >>  8];
        parsed[k++] = hex[(mm.cprlon & 0x000F0) >>  4];
        parsed[k++] = hex[(mm.cprlon & 0x0000F)];

    } else {
        if (decodeCPRrelative() < 0) {           // error decoding lat/lon
            //fo.distance = 0;
            //fo.bearing = 0;
            return false;
        }
        uint32_t y = (uint32_t)((111300.0 * 0.53996) * (fo.latitude - reflat)); // nm * 1000
        uint32_t x = (uint32_t)((111300.0 * 0.53996) * (fo.longitude - reflon) * CosLat(reflat));
        fo.distance = 0.001 * (float)iapproxHypotenuse1(x, y);
        int far = 1;
        if (fo.distance < 6.0)
            far = 0;
        else if (fo.distance > 30.0)
            far = 2;
        ++msg_by_dst_cat[far];
        // note that distance stats skip the too-far ones rejected earlier using maxcprdiff
        // filter by distance, but always include "followed" aircraft
        if (fo.addr != settings->follow) {
            if (fo.distance < (float)settings->minrange)
                return false;
            if (fo.distance > (float)settings->maxrange)
                return false;
        }
        if (fo.distance == 0) {
            fo.bearing = 0;
        } else {
            fo.bearing = iatan2_approx(y,x);
            if (fo.bearing < 0)
                fo.bearing += 360;
        }
        update_traffic_position();
    }

    parsedchars = k;
    return true;
}

static bool parse_velocity(bool justparse, char s)
{
    int k = parsedchars;

    if (justparse) {
        uint8_t b = ((msg[5] & 0xF8) >> 3);  // IC, IFR, NUC
        parsed[k++] = byte2hex_msb(b);
        parsed[k++] = byte2hex_lsb(b);
        parsed[k++] = s;
    }
    int ew_dir;
    int ew_velocity;
    int ns_dir;
    int ns_velocity;

    if (mm.sub == 1 || mm.sub == 2) {   // ground speed

      ew_dir = (msg[5]&4) >> 2;
      ew_velocity = ((msg[5]&3) << 8) | msg[6];
      ns_dir = (msg[7]&0x80) >> 7;
      ns_velocity = ((msg[7]&0x7f) << 3) | ((msg[8]&0xe0) >> 5);
      if (justparse) {
          // E-W velocity: 11 bits
          uint16_t v = (ew_dir << 10) | ew_velocity;
          parsed[k++] = hex[(v & 0x00F00) >> 8];
          parsed[k++] = hex[(v & 0x000F0) >> 4];
          parsed[k++] = hex[(v & 0x0000F)];
          parsed[k++] = s;
          // N-S velocity: 11 bits
          v = (ns_dir << 10) | ns_velocity;
          parsed[k++] = hex[(v & 0x00F00) >> 8];
          parsed[k++] = hex[(v & 0x000F0) >> 4];
          parsed[k++] = hex[(v & 0x0000F)];
          parsed[k++] = s;
      }
      if (ew_velocity > 0)        // zero means not available
          ew_velocity -= 1;
      if (ns_velocity > 0)
          ns_velocity -= 1;
      if (mm.sub == 2) {       // supersonic
          ew_velocity <<= 2;  // 4x
          ns_velocity <<= 2;  // 4x
      }

      if (ew_dir)
          fo.ewv = -ew_velocity;
      else
          fo.ewv = ew_velocity;
      if (ns_dir)
          fo.nsv = -ns_velocity;
      else
          fo.nsv = ns_velocity;

      // Compute velocity and angle from the two speed components
      fo.groundspeed = iapproxHypotenuse0(fo.nsv, fo.ewv);
      if (fo.groundspeed > 0) {
          fo.track = iatan2_approx(fo.nsv, fo.ewv);
          // We don't want negative values but a 0-360 scale.
          if (fo.track < 0)
              fo.track += 360;
          fo.track_is_valid = 1;
      } else {
          fo.track = 0;
          fo.track_is_valid=0;
      }

if(settings->debug>1)
Serial.printf("velocity: GS: %d, track: %d\n", fo.groundspeed, fo.track);

      // the following fields are absent from a groundspeed message type
      //fo.heading_is_valid=0; fo.heading=0; fo.airspeed_type=0; fo.airspeed=0;

    } else if (mm.sub == 3 || mm.sub == 4) {   // air speed (rare)

      fo.heading_is_valid = ((msg[5] & 4) >> 2);
      int16_t iheading = (((msg[5] & 3) << 5) | ((msg[6] >> 3) & 0x1F));
      //fo.heading = (360.0/128) * iheading;
      fo.heading = ((iheading * 360 + 180) >> 7);
      fo.airspeed_type = (((msg[7]) >> 7) & 1);
      fo.airspeed = ((msg[7]&0x7F) << 3) | (((msg[8]) >> 5) & 0x07);  // if 0, no info
      if (justparse) {
          // heading: 11 bits
          uint16_t v = (fo.heading_is_valid << 10) | iheading;
          parsed[k++] = hex[(v & 0x00F00) >> 8];
          parsed[k++] = hex[(v & 0x000F0) >> 4];
          parsed[k++] = hex[(v & 0x0000F)];
          parsed[k++] = s;
          // airspeed: 11 bits
          v = (fo.airspeed_type << 10) | fo.airspeed;
          parsed[k++] = hex[(v & 0x00F00) >> 8];
          parsed[k++] = hex[(v & 0x000F0) >> 4];
          parsed[k++] = hex[(v & 0x0000F)];
          parsed[k++] = s;
      }
      if (fo.airspeed > 0)        // zero means not available
          fo.airspeed -= 1;
      if (mm.sub == 4)
         fo.airspeed <<= 2; // 4x

if(settings->debug>1)
Serial.printf("velocity: AS: %d  heading: %d\n", fo.airspeed, fo.heading);

      // the following fields are absent from an airspeed message type
      //fo.ewv=0; fo.nsv=0; fo.groundspeed=0; fo.track=0; fo.track_is_valid=0;
    }

    fo.vert_rate_source = (msg[8]&0x10) >> 4;   // 0=GNSS, 1=baro
    int vert_rate_sign = (msg[8]&0x8) >> 3;
    int raw_vert_rate = ((msg[8]&7) << 6) | ((msg[9]&0xfc) >> 2);
    fo.vert_rate = ((raw_vert_rate - 1) << 6);    // * 64;
    if (vert_rate_sign)  fo.vert_rate = -fo.vert_rate;
    int raw_alt_diff = msg[10];  // MSB is sign
    int alt_diff_sign = ((raw_alt_diff & 0x80) >> 7);
    fo.alt_diff = ((raw_alt_diff & 0x7F) - 1) * 25;
    if (alt_diff_sign)  fo.alt_diff = -fo.alt_diff;  // GNSS altitude is below baro altitude

if(settings->debug>1)
Serial.printf("velocity: vert_rate: %d  alt_diff= %d\n", fo.vert_rate, fo.alt_diff);

    if (justparse) {
        // vertical rate: 11 bits
        uint16_t v = (fo.vert_rate_source << 10) | (vert_rate_sign << 9) | raw_vert_rate;
        parsed[k++] = hex[(v & 0x00F00) >> 8];
        parsed[k++] = hex[(v & 0x000F0) >> 4];
        parsed[k++] = hex[(v & 0x0000F)];
        parsed[k++] = s;
        // alt_diff: 8 bits
        parsed[k++] = byte2hex_msb(msg[10]);
        parsed[k++] = byte2hex_lsb(msg[10]);

    } else {
        update_traffic_velocity();
    }

    parsedchars = k;
    return true;
}

// decode altitude from surveillance responses (DF 4 and 20)
static bool parse_mode_s_altitude()
{
/*
byte 0-based:  0   1   2   3
bit, 0-based:  0   8   16  24-31
bit, 1-based:  1   9   17  25-32
               HH  HH  HH  HH
                       vv
                       17   21
                       BBBB BBBB BBBB BBBB
altitude bits:            ^ ^^^^ ^^^^ ^^^^
                                  M Q
*/
//if (settings->debug>1)
//Serial.printf("Mode S altitude msg: %s\n", buf);
    int n;
    if (msg[3]==0 && (msg[2] & 1) == 0) {  // altitude not available
        return false;
    } else if ((msg[3] & 0x40) != 0) {     // M bit set - metric altitude
        // N is the 12 bit integer resulting from the removal of the M bit
        n = ((msg[2]&0x1F)<<7) | ((msg[3]&0x80) >> 1) | (msg[3] & 0x3F);
        // convert altitude from meters into feet            
        n *= 3360;
        n >>= 10;
        // >>> this can't be right, since with 12 bits it is limited to 4095 meters
        n = 88888;
        //return false;
    } else if ((msg[3] & 0x10) == 0) {     // q_bit not set - high altitude
        n = 99999;
        //return false;
    } else {
        // N is the 11 bit integer resulting from the removal of M & Q bits
        n = ((msg[2]&0x1F)<<6) | ((msg[3]&0x80) >> 2) | ((msg[3]&0x20) >> 1) | (msg[3] & 0x0F);
        // altitude in feet
        n = n*25-1000;
    }
    fo.altitude = n;
if (settings->debug>1)
Serial.printf("Mode S altitude decoded: %d\n", n);
    return true;
}

// decode just the ICAO ID from all-call responses (DF 11)
static bool parse_all_call()
{
    fo.addr = (msg[1] << 16) | (msg[2] << 8) | msg[3];
if (settings->debug>1)
Serial.printf("all_call: heard from ICAO ID %X\n", fo.addr);
    if (settings->follow != 0 && fo.addr != settings->follow)
        return false;
    return true;
}

// assume the n chars in buf[] include the starting '*' but not the ending ';'

bool parse(char *buf, int n)
{
    fo = EmptyFO;   // start with a clean slate of all zeros
    mm = EmptyMsg;
    mm.msgtype = ' ';
    bool justparse = (settings->parsed == FLDFMT);
    char s = (settings->format==TXTFMT? ' ' : settings->format==TABFMT? '\t' : ',');
    parsedchars = 0;
    int k=0;
    int i=1;
    if (buf[0] == '*') {
        if (n != 29 && (settings->dfs != DFNOTL && settings->dfs != DF20 && settings->dfs != DFSALL))
            return false;     // not a 112-bit ES
        if (justparse) {
            parsed[0] = buf[1];    // DF & CA
            parsed[1] = buf[2];
            parsed[2] = s;
            k = 3;
        }
        fo.rssi = 0;
    } else
    if (buf[0] == '+') {
        if (n != 31 && (settings->dfs != DFNOTL && settings->dfs != DF20 && settings->dfs != DFSALL))
            return false;     // not a 112-bit ES
        fo.rssi = (hex2bin(buf[1]) << 4) | hex2bin(buf[2]);
        int rssi_ = fo.rssi;
        // between 0x18=24 (weak) and 0x2d=45 (strongest signals) - subtract 22
        if (rssi_ < 22)  rssi_ = 22;
        if (rssi_ > 46)  rssi_ = 46;
        ++msg_by_rssi[rssi_-22];
        i = 3;    // point to DF
        if (justparse) {
            parsed[0] = buf[1];    // rssi
            parsed[1] = buf[2];
            parsed[2] = s;
            parsed[3] = buf[3];    // DF & CA
            parsed[4] = buf[4];
            parsed[5] = s;
            k = 6;
        }
    }
    else
        return false;     // not a valid GNS5892 sentence

    if (justparse) {
        int b=i+2;        // point to ICAO ID
        for (int m=0; m<6; m++)
            parsed[k++] = buf[b++];
        parsed[k++] = s;
    }

    // parse just the first 4 bytes for now
//    msg[0] = ((hex2bin(buf[i])) << 4) | hex2bin(buf[i+1]);
//    i += 2;      // 2 hex chars converted into one binary byte
    int j=0;
    while (j < 4) {
        msg[j++] = (hex2bin(buf[i]) << 4) | hex2bin(buf[i+1]);
        i += 2;
    }

    mm.frame = msg[0]>>3;    // Downlink Format
    if (mm.frame > 22)
        mm.frame = 22;
    ++msg_by_DF[mm.frame];
    ++msg_by_hour[ourclock.hour];

    int dfs = settings->dfs;
    if (mm.frame == 17) {
        if (dfs == DF18 || dfs == DF20)
            return false;
    } else if (mm.frame == 18) {
        if (dfs == DF17 || dfs == DF20)
            return false;
    } else if (dfs == DF20 || dfs == DFSALL || dfs == DFNOTL) {
        // and neither 17 nor 18
        if (mm.frame == 11) {
            if (dfs == DFSALL) {
                mm.msgtype = 'L';
                return parse_all_call();    // all-call responses - just ID
            }
            return false;
        } else if (mm.frame == 4) {
            mm.msgtype = 'A';
            return parse_mode_s_altitude();
        } else if (mm.frame == 20) {
            mm.msgtype = 'B';
            return parse_mode_s_altitude();
        } else if (mm.frame == 16) {
            if (dfs == DFSALL || dfs == DFNOTL) {   // but not DF20
                mm.msgtype = 'C';
                return parse_mode_s_altitude();
            }
            return false;
        } else if (mm.frame == 0) {
            if (dfs == DFSALL || dfs == DFNOTL) {
                mm.msgtype = 'S';
                return parse_mode_s_altitude();
            }
            return false;
        }
    } else {                // dfs == D17,D18,D78
        return false;
    }
    // at this point only DF17 and DF18 are being processed

    // convert the rest of the message from hex to binary
    if (! settings->chk_crc)
        n -= 6;               // skip the PI (checksum field)
    j=4;
    while (i < n) {
        msg[j++] = (hex2bin(buf[i]) << 4) | hex2bin(buf[i+1]);
        i += 2;
    }

    if (settings->chk_crc) {
        if (check_crc() == false) {
            ++msg_by_crc_cat[1];
            return false;
        }
        ++msg_by_crc_cat[0];
    }

    //ca = msg[0] & 7;        // Responder capabilities.

    // ICAO address
    fo.addr = (msg[1] << 16) | (msg[2] << 8) | msg[3];

    // filter by identity
    if (settings->follow != 0 && fo.addr != settings->follow)
        return false;

    // parsing of the 56-bit ME - just DF 17-18:
    // DF19 is military, encrypted, show something anyway?
    //if (mm.frame == 19) ...

    mm.type = msg[4] >> 3;   // Extended squitter message type.
    mm.sub = msg[4] & 7;     // Extended squitter message subtype.
    if (justparse) {
        parsed[k++] = byte2hex_msb(mm.type);
        parsed[k++] = byte2hex_lsb(mm.type);
        parsed[k++] = '-';
        parsed[k++] = hex[mm.sub];
        parsed[k++] = s;
        parsedchars = k;
    }

    if (mm.type >= 1 && mm.type <= 4) {

        mm.msgtype = 'I';  // Aircraft Identification and Category
        return parse_identity(justparse, s);

    } else if (mm.type >= 9 && mm.type <= 22 && mm.type != 19) {

        mm.msgtype = 'P';  // Airborne position Message
        return parse_position(justparse, s);

    } else if (mm.type == 19 && mm.sub >= 1 && mm.sub <= 4) {

        mm.msgtype = 'V';  // Airborne Velocity Message
        return parse_velocity(justparse, s);

    }

    return false;
}
