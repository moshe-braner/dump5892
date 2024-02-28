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

int bytes_per_ms = (SERIAL_OUT_BR >> 13);  // output rate - allows 12 serial bits per byte
static bool has_serial2 = false;

void setup()
{
  Serial.setTxBufferSize(OUTPUT_BUF_SIZE);  // needs to be done before begin()!
  Serial.begin(SERIAL_OUT_BR, SERIAL_8N1);
  delay(50);

  Serial.println();
  Serial.print(F(FIRMWARE_IDENT));
  Serial.print(F(" version: "));
  Serial.println(F(FIRMWARE_VERSION));
  Serial.println(F("Copyright (C) 2024 Moshe Braner"));
  Serial.flush();

  EEPROM_setup();
  minrange10 = 10 * settings->minrange;
  maxrange10 = 10 * settings->maxrange;

  if (settings->outbaud) {
    Serial.printf("switching output to %d baud rate\n", HIGHER_OUT_BR);
    delay(200);
    Serial.end();
    delay(500);
    Serial.setTxBufferSize(OUTPUT_BUF_SIZE);
    Serial.begin(HIGHER_OUT_BR, SERIAL_8N1);
    bytes_per_ms = (HIGHER_OUT_BR >> 13);
  }

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
      has_serial2 = true;
  }

  timenow = 100;
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

static void in_discard()
{
    Serial.println("...");
    ++in_discards;
}

// this is not actually used:
static void out_discard()
{
    Serial.println(".");
    ++out_discards;
}

// estimate the serial output buffer state by the number of bytes sent and when,
//   since Serial.availableForWrite() is broken in Arduino ESP32 core v2.0.3
//      (see https://github.com/espressif/arduino-esp32/issues/6697)
static bool output_maybe(char *p, int n)
{
    static int output_bytes = 0;
    static uint32_t last_output_ms = 0;
    uint32_t now_ms = millis();
    int sent_since;                  // potentially sent over the time interval
    if (now_ms == last_output_ms)    // less than a millisec
        sent_since = bytes_per_ms;   // don't get stuck
    else
        sent_since = (now_ms - last_output_ms) * bytes_per_ms;
    if (output_bytes < sent_since || Serial.availableForWrite() == 128)
        output_bytes = 0;   // buffer flushed
    else
        output_bytes -= sent_since;
    last_output_ms = now_ms;
    if (output_bytes + n > (OUTPUT_BUF_SIZE-128)) {
        // presumably not enough room in buffer
        Serial.println(".");
        ++out_discards;
        return false;
    }
    Serial.write(p, n);
    output_bytes += n;
    return true;
}

void output_raw()
{
    buf[inputchars++] = ';';
    buf[inputchars++] = '\r';
    buf[inputchars++] = '\n';
    output_maybe(buf, inputchars+3);
    inputchars = 0;      // start a new input sentence
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
    //if (mm.frame != 17 && mm.frame != 18)
    //    return;
    // construct a single line of text about the last arrived message
    const char *cs = ((fo.callsign[0] != '\0' || settings->format!=TXTFMT)? fo.callsign : "        ");
    const char *fmt;
    if (settings->dstbrg) {
      if (settings->format==TABFMT)
        fmt = "%s\t%02d\t%02d\t%c\t%06X\t%s\t%d\t%.1f\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n";
      else if (settings->format==CSVFMT)
        fmt = "%s,%02d,%02d,%c,%06X,%s,%d,%.1f,%d,%d,%d,%d,%d,%d,%d,%d\r\n";
      else // TXTFMT
        fmt = "%s %02d %02d %c %06X %s %02d %5.1f %3d %5d %5d %5d %4d %4d %3d %3d\r\n";
            //tm rssi DF msgtyp ID cs actyp dst  brg altitud altdif vs nsv ewv aspd hdg
      snprintf(parsed, PARSE_BUF_SIZE, fmt,
        time_string(true),
        fo.rssi, mm.frame, mm.msgtype, fo.addr, cs, fo.aircraft_type,
        (fo.distance==0? 0.1*(float)fo.approx_dist : fo.distance),
        (fo.distance==0? fo.approx_brg  : fo.bearing),
        fo.altitude, fo.alt_diff, fo.vert_rate,
        fo.nsv, fo.ewv, fo.airspeed, fo.heading);
    } else {
      if (settings->format==TABFMT)
        fmt = "%s\t%d\t%d\t%c\t%06X\t%s\t%d\t%.4f\t%.4f\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n";
      else if (settings->format==CSVFMT)
        fmt = "%s,%d,%d,%c,%06X,%s,%d,%.4f,%.4f,%d,%d,%d,%d,%d,%d,%d\r\n";
      else // TXTFMT
        fmt = "%s %02d %02d %c %06X %s %02d %9.4f %9.4f %5d %5d %5d %4d %4d %3d %3d\r\n";
             //tm rssi DF msgtyp ID cs actyp lat lon altitud altdif vs nsv ewv aspd hdg
      snprintf(parsed, PARSE_BUF_SIZE, fmt,
        time_string(true),
        fo.rssi, mm.frame, mm.msgtype, fo.addr, cs, fo.aircraft_type,
        fo.latitude, fo.longitude, fo.altitude, fo.alt_diff, fo.vert_rate,
        fo.nsv, fo.ewv, fo.airspeed, fo.heading);
    }
    parsedchars = strlen(parsed);
    output_maybe(parsed, parsedchars);
}

void output_page()
{
    // only output a page if there is new data from the followed aircraft
    int i;
    if (settings->follow != 0) {
        i = find_traffic_by_addr(settings->follow);
        if (i == 0)
            return;
        --i;      // from base-1 to base-0 indexing
    } else {
        for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
           if (container[i].addr)                    // the (sole) tracked aircraft
               break;
        }
        if (i == MAX_TRACKING_OBJECTS)               // should not happen
            return;
    }
    traffic_update(i);
    ufo_t *fop = &container[i];
    if (fop->reporttime >= fop->positiontime)        // nothing new to report
        return;
    if (timenow < fop->reporttime + 3)              // don't report too often
        return;
    fop->reporttime = timenow;
    Serial.println("\n----------------------------------------\n");
    // construct a page of text about the followed aircraft
    uint32_t timesince = timenow - fop->positiontime;
    const char *cs = ((fo.callsign[0] != '\0')? fo.callsign : "        ");
    snprintf(parsed, PARSE_BUF_SIZE,
"\
%s      %d seconds since last position report    RSSI=%02d\n\
ICAO ID: %06X   Callsign: %s    Aircraft Type: %s\n\
Latitude = %9.4f   Longitude = %9.4f\n\
     - From here:  %5.1f nm,   %d bearing\n\
Altitude = %5d (%s) (GNSS altitude rel to baro altitude: %d)\n\
Vertical speed = %d fpm\n\
Groundspeed = %4d knots   Track   = %3d\n\
Airspeed    = %4d knots   Heading = %3d\n",
        time_string(true), timesince, fop->rssi,
        fop->addr, cs, ac_type_label[fop->aircraft_type],
        fop->latitude, fop->longitude,
        (fop->distance==0? 0.1*(float)fop->approx_dist : fop->distance),
        (fop->distance==0? fop->approx_brg  : fop->bearing),
        fop->altitude, (fop->alt_type? "GNSS" : "barometric"),
        fop->alt_diff, fop->vert_rate,
        fop->groundspeed, fop->track,
        fop->airspeed, fop->heading);
    parsedchars = strlen(parsed);
    Serial.write(parsed, parsedchars);  // may block, but this happens only every 3 seconds or so
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
      nexttime = millis() + 4000;
      if (num_tracked == 0)
          return;                // try again in another 4 seconds
      active = true;
      tick = 0;
//      Serial.println("\n----------------------------------------\n");
      Serial.println("");
  } else {
      // active, report one aircraft per loop() iteration (no millis() wait)
      tick++;
      if (tick >= MAX_TRACKING_OBJECTS) {
          active = false;
          return;
      }
  }
  ufo_t *fop = &container[tick];
  if (fop->addr == 0)
      return;
  if (timenow > fop->positiontime + 3)     // not heard from recently
      return;
  if (timenow < fop->reporttime + 2)       // reported recently
      return;
  fop->reporttime = timenow;
  // construct a single line of text about each tracked aircraft
  const char *fmt;
  const char *t = time_string(true);      // a bit later than actual position time
  const char *cs = ((fo.callsign[0] != '\0' || settings->format!=TXTFMT)? fo.callsign : "        ");
  const char *g = (fop->alt_type? "g" : settings->format==TXTFMT? " " : "");
  if (settings->dstbrg) {
    if (settings->format==TABFMT)
      fmt = "[%d]\t%s\t%d\t%06X\t%s\t%d\t%.1f\t%d\t%s\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n";
    else if (settings->format==CSVFMT)
      fmt = "[%d],%s,%d,%06X,%s,%d,%.1f,%d,%s,%d,%d,%d,%d,%d,%d,%d\r\n";
    else // TXTFMT
      fmt = "[%2d] %s %02d %06X %s %02d %5.1f %3d %s %5d %5d %5d %3d %3d %3d %3d\r\n";
           //idx time rssi ID cs actyp dst brg altitude altdif vs gspd trk aspd hdg
    snprintf(parsed, PARSE_BUF_SIZE, fmt,
      tick, t, fop->rssi, fop->addr, cs, fop->aircraft_type,
      (fop->distance==0? 0.1*(float)fop->approx_dist : fop->distance),
      (fop->distance==0? fop->approx_brg  : fop->bearing),
      g, fop->altitude, fop->alt_diff, fop->vert_rate,
      fop->groundspeed, fop->track,
      fop->airspeed, fop->heading);
  } else {
    if (settings->format==TABFMT)
      fmt = "[%d]\t%s\t%d\t%06X\t%s\t%d\t%.4f\t%.4f\t%s\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n";
    else if (settings->format==CSVFMT)
      fmt = "[%d],%s,%d,%06X,%s,%d,%.4f,%.4f,%s,%d,%d,%d,%d,%d,%d,%d\r\n";
    else // TXTFMT
      fmt = "[%2d] %s %02d %06X %s %02d %9.4f %9.4f %s %5d %5d %5d %3d %3d %3d %3d\r\n";
            //idx time rssi ID cs actyp lat lon altitud altdif vs gspd trk aspd hdg
    snprintf(parsed, PARSE_BUF_SIZE, fmt,
      tick, t, fop->rssi, fop->addr, cs, fop->aircraft_type,
      fop->latitude, fop->longitude,
      g, fop->altitude, fop->alt_diff, fop->vert_rate,
      fop->groundspeed, fop->track,
      fop->airspeed, fop->heading);
  }
  parsedchars = strlen(parsed);
  if (output_maybe(parsed, parsedchars) == false) {
      // try the same one again next time around the loop():
      fop->reporttime -= 2;
      --tick;
  }
}

static bool input_complete;

void input_loop()
{
    if (input_complete)
        inputchars = 0;          // start a new input sentence
    input_complete = false;
    if (has_serial2 == false);
        return;
    int n = inputchars;
    if (n == 0) {                // waiting for a new sentence to start
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
    if (! input_complete) {
        parsing_success = false;
        return;
    }

    if (inputchars > 0 && settings->parsed != RAWFMT) {
        if (buf[0] == '*' || buf[0] == '+') {        // ADS-B data received
            parsing_success = parse(buf, inputchars);
        } else if (buf[0] == '#') {                  // response to commands
            Serial.write(buf, inputchars);           // copy to console
            Serial.println("");
        }
    }
}

void output_loop()
{
    if (settings->parsed == NOTHING)
        return;
    // output formats based on the traffic table:
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
    // output formats based on the most recent message:
    if (! input_complete)
        return;
    if (inputchars == 0)
        return;
    if (settings->parsed == RAWFMT) {
        output_raw();
        return;
    }
    // output formats based on the most recent parsed message:
    if (parsing_success == false)
        return;
    parsing_success = false;
    if (settings->parsed == RAWFILT) {   // same as RAW but filtered
        int i = find_traffic_by_addr(fo.addr);
        if (i == 0)                              // not in traffic table
            return;                              // have not received a position message yet
        if (container[i-1].positiontime == 0)    // no identity message, or filtered out 
            return;
        output_raw();
        return;
    }
    inputchars = 0;
    if (settings->parsed == FLDFMT) {
        output_maybe(parsed, parsedchars);
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
                // any additional chars will be accepted but ignored
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
