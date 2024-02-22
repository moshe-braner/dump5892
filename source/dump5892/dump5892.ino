/*
 * dump5892.ino
 *
 * Copyright (C) 2024 Moshe Braner
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version - see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include "dump5892.h"

void setup()
{
  Serial.setTxBufferSize(OUTPUT_BUF_SIZE);  // or should this be done after begin()?
  Serial.begin(SERIAL_OUT_BR, SERIAL_8N1);

  Serial.println();
  Serial.print(F(FIRMWARE_IDENT));
  Serial.print(F(" version: "));
  Serial.println(F(FIRMWARE_VERSION));
  Serial.println(F("Copyright (C) 2024 Moshe Braner"));
  Serial.flush();

  EEPROM_setup();
  minrange10 = 10 * settings->minrange;
  maxrange10 = 10 * settings->maxrange;

  traffic_setup();

  CPRRelative_setup();
  if (reflat == 0 || reflon == 0)
      Serial.println("\n>>>> Reference lat/lon not set, positions will be wrong!\n");

  if (settings->rx_pin > 39) {
      Serial.println("\n>>>> Invalid RX GPIO pin, not starting Serial2\n");
  } else if (settings->tx_pin > 33) {
      Serial.println("\n>>>> Invalid TX GPIO pin, not starting Serial2\n");
  } else {
      Serial2.setRxBufferSize(INPUT_BUF_SIZE);
      Serial2.begin(SERIAL_IN_BR, SERIAL_8N1, settings->rx_pin, settings->tx_pin);
  }
  delay(1000);
  pause5892();
  show_settings();
  reset5892();
}

char *time_string(bool withdate)
{
    static char s[20];    // at most: yyyy/mm/dd hh:mm:ss
    if (withdate && ourclock.year) {
        snprintf(s,20,"20%02d/%02d/%02d %02d:%02d:%02d",
             ourclock.year,ourclock.month,ourclock.day,
             ourclock.hour,ourclock.minute,ourclock.second);
    } else {
        snprintf(s,20,"%02dd%02dh%02dm%02ds",
             ourclock.day,ourclock.hour,ourclock.minute,ourclock.second);
    }
    return s;
}

void in_discard()
{
    Serial.println("...");
    ++in_discards;
}

void out_discard()
{
    Serial.println(".");
    ++out_discards;
}

/*
ALL
    IDENTITY
        POSITION
            VELOCITY
                GNSS
                    AIR
rssi
DF
ID
    callsign
type
subtype
    aircraft_type
        lat
        lon
        alt_type
        altitude
            vert_rate
            alt_diff
                nsv
                ewv
                groundspeed
                track
                    airspeed
                    heading
*/
void output_decoded()
{
    if (mm.frame != 17 && mm.frame != 18)
        return;
    const char *s = (settings->format==TXTFMT? " " : settings->format==TABFMT? "\t" : ",");
    // construct a single line of text about the last arrived message
    if (settings->dstbrg) {
      snprintf(parsed, PARSE_BUF_SIZE,
        "%s%s%02d%s%02d%s%06X%s%s%s%02d%s%5.1f%s%3d%s%s%s%5d%s%3d%s%4d%s%3d%s%3d%s%3d%s%3d\r\n",
        //tm rssi  DF    ID    cs  actyp  dst    brg  altitud altdif vs nsv ewv aspd  hdg
        time_string(true), s,
        fo.rssi, s, mm.frame, s, fo.addr, s, fo.callsign, s,
        fo.aircraft_type, s,
        (fo.distance==0? 0.1*(float)fo.approx_dist : fo.distance),s,
        (fo.distance==0? fo.approx_brg  : fo.bearing),s,
        (fo.alt_type? "g" : "b"), s, fo.altitude, s,
        fo.alt_diff, s, fo.vert_rate, s,
        fo.nsv, s, fo.ewv, s,
        fo.airspeed, s, fo.heading);
    } else {
      snprintf(parsed, PARSE_BUF_SIZE,
        "%s%s%02d%s%02d%s%06X%s%s%s%02d%s%9.4f%s%9.4f%s%s%s%5d%s%3d%s%4d%s%3d%s%3d%s%3d%s%3d\r\n",
        //tm rssi  DF    ID    cs  actyp  lat    lon    altitud altdif vs nsv ewv aspd  hdg
        time_string(true), s,
        fo.rssi, s, mm.frame, s, fo.addr, s, fo.callsign, s,
        fo.aircraft_type, s,
        fo.latitude, s, fo.longitude, s, (fo.alt_type? "g" : "b"), s, fo.altitude, s,
        fo.alt_diff, s, fo.vert_rate, s,
        fo.nsv, s, fo.ewv, s,
        fo.airspeed, s, fo.heading);
    }
    parsedchars = strlen(parsed);
    if (Serial.availableForWrite() > parsedchars)
        Serial.write(parsed, parsedchars);
    else   // discard this sentence, let slower output catch up
        out_discard();
}

void output_page()
{
    // only output a page if there is new data from the followed aircraft
    int i;
    if (settings->follow != 0) {
        i = find_traffic_by_addr(settings->follow);
        if (i == 0)
            return;
        --i;
    } else {
        for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
           if (container[i].addr)
               break;
        }
        if (i == MAX_TRACKING_OBJECTS)
            return;
    }
    traffic_update(i);
    ufo_t *fop = &container[i];
    if (timenow < fop->reporttime + 3)
        return;
    fop->reporttime = timenow;
    Serial.println("\n----------------------------------------\n");
    // construct a page of text about the followed aircraft
    uint32_t timesince = timenow - fop->positiontime;
    snprintf(parsed, PARSE_BUF_SIZE,
"\
%s      %d seconds since last position report\n\
ICAO %06X  Callsign %s  Aircraft Type: %s  RSSI=%02d  DF%02d\n\
Latitude = %9.4f   Longitude = %9.4f\n\
     - From here:  %5.1 nm, %d bearing\n\
Altitude = %5d (%s) (GNSS altitude rel to baro altitude: %3d)\n\
Vertical speed = %5d fpm\n\
Groundspeed = %4d knots   Track   = %3d\n\
Airspeed    = %4d knots   Heading = %3d\n",
        time_string(true), timesince,
        fop->addr, fop->callsign, ac_type_label[fop->aircraft_type], fop->rssi, mm.frame,
        (fop->distance==0? 0.1*(float)fop->approx_dist : fop->distance),
        (fop->distance==0? fop->approx_brg  : fop->bearing),
        fop->latitude, fop->longitude, fop->altitude, (fop->alt_type? "GNSS" : "barometric"),
        fop->alt_diff, fop->vert_rate,
        fop->groundspeed, fop->track,
        fop->airspeed, fop->heading);
    parsedchars = strlen(parsed);
    if (Serial.availableForWrite() > parsedchars)
        Serial.write(parsed, parsedchars);
    else   // discard this sentence, let slower output catch up
        out_discard();
    delay(50);
}

// list active entries in traffic table (those with recent position data)
void output_list()
{
  static bool active = false;
  static int tick;
  static uint32_t nexttime = 0;
  if (! active) {
      if (millis() < nexttime)
          return;
      nexttime = millis() + 4000;   // report every 4 seconds
      if (num_tracked == 0)
          return;                  // try again in another 4 seconds
      active = true;
      tick = 0;
//      Serial.printf("\n----------------------------------------\n\n%s\n\n",
//         time_string(true));
      Serial.println("\n----------------------------------------\n");
  } else {                   // active, one aircraft per loop() iteration
      tick++;
      if (tick >= MAX_TRACKING_OBJECTS) {
          active = false;
          return;
      }
  }
  const char *s = (settings->format==TXTFMT? " " : settings->format==TABFMT? "\t" : ",");
  ufo_t *fop = &container[tick]; 
  if (fop->addr == 0)
      return;
  if (timenow > fop->positiontime + 2)     // not heard from recently
      return;
  // construct a single line of text about each tracked aircraft
  char *t = time_string(true);    // a bit later than actual position time
  if (settings->dstbrg) {
Serial.printf("approx dst/brg: %d %d", fop->approx_dist, fop->approx_brg);
Serial.printf("other  dst/brg: %.1f %d", fop->distance, fop->bearing);
    snprintf(parsed, PARSE_BUF_SIZE,
      "[%2d]%s%s%s%02d%s%06X%s%s%s%02d%s%5.1f%s%3d%s%s%s%5d%s%3d%s%4d%s%3d%s%3d%s%3d%s%3d\r\n",
      //idx time  rssi ID   cs  actyp  dst    brg  altitud  altdif vs  gspd trk aspd  hdg
      tick,s, t,s, fop->rssi,s, fop->addr,s, fop->callsign,s, fop->aircraft_type,s,
      (fop->distance==0? 0.1*(float)fop->approx_dist : fop->distance),s,
      (fop->distance==0? fop->approx_brg  : fop->bearing),s,
      (fop->alt_type? "g" : "b"),s, fop->altitude,s,
      fop->alt_diff,s, fop->vert_rate,s,
      fop->groundspeed,s, fop->track,s,
      fop->airspeed,s, fop->heading);
  } else {
    snprintf(parsed, PARSE_BUF_SIZE,
      "[%2d]%s%s%s%02d%s%06X%s%s%s%02d%s%9.4f%s%9.4f%s%s%s%5d%s%3d%s%4d%s%3d%s%3d%s%3d%s%3d\r\n",
      //idx time  rssi ID   cs  actyp  lat    lon    altitud  altdif vs  gspd trk aspd  hdg
      tick,s, t,s, fop->rssi,s, fop->addr,s, fop->callsign,s, fop->aircraft_type,s,
      fop->latitude,s, fop->longitude,s, (fop->alt_type? "g" : "b"),s, fop->altitude,s,
      fop->alt_diff,s, fop->vert_rate,s,
      fop->groundspeed,s, fop->track,s,
      fop->airspeed,s, fop->heading);
  }
  parsedchars = strlen(parsed);
  if (Serial.availableForWrite() > parsedchars)
      Serial.write(parsed, parsedchars);
  else   // discard this sentence, let slower output catch up
      out_discard();
}

static bool input_complete;

void input_loop()
{
    int n = inputchars;
    input_complete = false;
    if (n == 0) {                  // waiting for a new sentence to start
        if (Serial2.available() > (INPUT_BUF_SIZE - 256)) {
            // input buffer is getting full, drain it
            Serial2.readBytes(buf, 256);  // discard some input data
            in_discard();
        }
    }
    while (Serial2.available()) {  // loop until a full sentence or no more data
        char c = Serial2.read();
        if (c=='*' || c=='+' || c=='#') {
            buf[0] = c;            // start new sentence, drop any preceding data
            n = 1;
        } else if (n == 0) {      // wait for a valid starting char
            continue;
        } else if (c==';' || c=='\r' || c=='\n') {   // completed sentence
            if (n > 14 /* && n <= 32 */ ) {
                input_complete = true;
                break;
            }
            n = 0;                 // invalid, start over
        } else {
            buf[n++] = c;
        }
    }
    inputchars = n;
}

void parse_loop()
{
    if (! input_complete)
        return;

    if (inputchars > 0 && settings->parsed != RAWFMT) {
        if (buf[0] == '*' || buf[0] == '+') {        // ADS-B data received
            parsing_success = parse(buf, inputchars);
        } else if (buf[0] == '#') {                  // response to commands
            Serial.write(buf, inputchars);           // copy to console
            Serial.println("");
        }
        inputchars = 0;      // start a new input sentence
    }
}

void output_loop()
{
    if (settings->parsed == NOTHING)
        return;
    if (settings->parsed == LSTFMT) {
        output_list();
        return;
    }
    if (settings->parsed == PAGEFMT) {
        if (settings->follow != 0 || num_tracked == 1)
            output_page();
        else
            output_list();
        return;
    }
    if (! input_complete)
        return;
    if (inputchars > 0 && settings->parsed == RAWFMT) {
        buf[inputchars] = '\0';
        if (Serial.availableForWrite() > inputchars)
            Serial.write(buf, inputchars);
        else   // discard this sentence, let slower output catch up
            out_discard();
        inputchars = 0;      // start a new input sentence
        return;
    }
    if (parsing_success == false)
        return;
    parsing_success = false;
    if (settings->parsed == FLDFMT) {
        if (Serial.availableForWrite() > parsedchars)
            Serial.write(parsed, parsedchars);
        else   // discard this sentence, let slower output catch up
            out_discard();
        return;
    }
    output_decoded();
}

void cmd_loop()
{
    bool complete = false;
    int end_of_cmd = 0;
    while (Serial.available()) {      // loop until no more data
        char c = Serial.read();
        if (c==';' || c=='\r' || c=='\n') {   // completed sentence
            if (! complete) {
                end_of_cmd = cmdchars;        // excludes the ';'
                complete = true;
            }
        }
        if (cmdchars >= 120) {
            cmdchars = 0;
            complete = false;
        }
        cmdbuf[cmdchars++] = c;
    }
    if (complete) {
        cmdbuf[end_of_cmd] = '\0';
        if (cmdbuf[0] == '*' || cmdbuf[0] == '+') {   // simulated ADS-B data
            if (end_of_cmd > 3) {
                strcpy(buf, cmdbuf);
                inputchars = end_of_cmd;
                input_complete = true;
                parse_loop();
                traffic_loop();
                output_loop();
            }
        } else {
            interpret_cmd(cmdbuf, end_of_cmd);
        }
        cmdchars = 0;
    }
}

void clock_loop()
{
    uint32_t ms = millis();
    if (ms >= ourclock.nextsecond) {
        if (ourclock.nextsecond == 0)
            ourclock.nextsecond = ms;
        ourclock.nextsecond += 1000;
        ++timenow;
        ++ourclock.second;
        if (ourclock.second == 60) {
            ourclock.second = 0;
            ++ourclock.minute;
            if (ourclock.minute == 60) {
                ourclock.minute = 0;
                ++ourclock.hour;
                if (ourclock.hour == 24) {
                    ourclock.hour = 0;
                    ++ourclock.day;
                    // ignore what happens at the end of the month
                }
            }
        }
    }
}

void loop()
{
  input_loop();
  yield();
  parse_loop();
  yield();
  traffic_loop();
  yield();
  output_loop();
  yield();
  cmd_loop();
  clock_loop();
  yield();
}
