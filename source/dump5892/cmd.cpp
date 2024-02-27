/*
 * cmd.cpp
 * Copyright (C) 2024 Moshe Braner
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version - see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>

#include "dump5892.h"

static void toggle_baud_rate()
{
    int baudrate;
    if (settings->baud2) {
        settings->baud2 = 0;
        baudrate = SERIAL_OUT_BR;
    } else {
        settings->baud2 = 1;
        baudrate = HIGHER_OUT_BR;
    }
    Serial.printf("switching output to %d baud rate...\n", baudrate);
    delay(1000);
    Serial.end();
    delay(2000);
    Serial.setTxBufferSize(OUTPUT_BUF_SIZE);
    Serial.begin(baudrate, SERIAL_8N1);
    Serial.println("... hello again!");
}

static void send5892(const char *cmd)
{
    Serial2.print(cmd);
    Serial2.print("\r");
}

void play5892()
{
if(settings->debug)
Serial.println("\n(un-paused)");
else
    Serial.println("");
    if (settings->incl_rssi) {
        if (settings->dfs == ALLDFS) send5892("#49-82");
        else                         send5892("#49-83");
    } else {
        if (settings->dfs == ALLDFS) send5892("#49-02");
        else                         send5892("#49-03");
    }
    paused = false;
}

void pause5892()
{
    send5892("#49-00");
    paused = true;
if(settings->debug)
Serial.println("\n(paused)");
else
    Serial.println("");
}

static void play_pause()
{
    if (paused)  play5892();
    else         pause5892();
}

void reset5892()
{
    pause5892();
    Serial.println("> resetting GNS5892...");
    delay(500);
    send5892("#FF");
    delay(1000);
}

static void show_columns()
{
  if (settings->parsed == DECODED) {
    if (settings->format==TABFMT) {
      if (settings->dstbrg)
        Serial.println(">time\trssi\tDF\tmsgtyp\tID\tcallsign\tactyp\tdist\tbrg\taltitud\taltdif\tvs\tnsv\tewv\taspd\thdg");
      else
        Serial.println(">time\trssi\tDF\tmsgtyp\tID\tcallsign\tactyp\tlat\tlon\taltitud\taltdif\tvs\tnsv\tewv\taspd\thdg");
    } else if (settings->format==CSVFMT) {
      if (settings->dstbrg)
        Serial.println(">time,rssi,DF,msgtyp,ID,callsign,actyp,dist,brg,altitud,altdif,vs,nsv,ewv,aspd,hdg");
      else
        Serial.println(">time,rssi,DF,msgtyp,ID,callsign,actyp,lat,lon,altitud,altdif,vs,nsv,ewv,aspd,hdg");
    } else {
      if (settings->dstbrg)
        Serial.println(">time rssi DF msgtyp ID  callsign  actyp  dist   brg   altitud altdif vs nsv ewv aspd  hdg");
      else
        Serial.println(">time rssi DF msgtyp ID  callsign  actyp  lat    lon   altitud altdif vs nsv ewv aspd  hdg");
    }
  } else if (settings->parsed == LSTFMT) {
    if (settings->format==TABFMT) {
      if (settings->dstbrg)
        Serial.println(">index\ttime\trssi\tID\tcallsign\tactyp\tdst\tbrg\tg\taltitude\taltdif\tvs\tgspd\ttrk\taspd\thdg");
      else
        Serial.println(">index\ttime\trssi\tID\tcallsign\tactyp\tlat\tlon\tg\taltitude\taltdif\tvs\tgspd\ttrk\taspd\thdg");
    } else if (settings->format==CSVFMT) {
      if (settings->dstbrg)
        Serial.println(">index,time,rssi,ID,callsign,actyp,dst,brg,g,altitude,altdif,vs,gspd,trk,aspd,hdg");
      else
        Serial.println(">index,time,rssi,ID,callsign,actyp,lat,lon,g,altitude,altdif,vs,gspd,trk,aspd,hdg");
    } else {
      if (settings->dstbrg)
        Serial.println(">index time rssi ID callsign actyp dst brg g altitude altdif vs gspd trk aspd hdg");
      else
        Serial.println(">index time rssi ID callsign actyp lat lon g altitude altdif vs gspd trk aspd hdg");
    }
  } else if (settings->parsed == FLDFMT) {
    Serial.println(">(rssi) DF CA ID 01..04(actype) callsign");
    Serial.println(">(rssi) DF CA ID 09..12,14..16-x alt tflag fflag cprlat cprlon");
    Serial.println(">(rssi) DF CA ID 13-1..4 [IC,IFR,NUC] ewv nsv (or hdg aspd) vert altdif");
  }
}

void show_settings()
{
    Serial.print("Date and time set to: ");
    Serial.println(time_string(true));
    char types[64];
    char IDs[64];
    if (settings->ac_type == 0)
        snprintf(types,64,"Show all ('0') aircraft types");
    else
        snprintf(types,64,"Only show aircraft type %d", settings->ac_type);
    if (settings->follow == 0)
        snprintf(IDs,64,"Show all ('0') aircraft IDs");
    else
        snprintf(IDs,64,"Only show aircraft ID %06X", settings->follow);
Serial.printf("\n\
CURRENT SETTINGS:\n\n\
LOC - reference location, lat/lon: %.5f %.5f\n\
TIM - current date and time: %s\n\
Output format:\n\
    %s\n\
    %s\n\
    BRG - Show %s\n",
reflat, reflon, time_string(true),
(settings->parsed==NOTHING? "NON - do not export detailed data" :
 settings->parsed==RAWFMT?  "RAW - export raw hex format" :
 settings->parsed==RAWFILT? "FIL - export raw, but filtered" :
 settings->parsed==FLDFMT?  "FLD - export hex format with fields separated" :
 settings->parsed==DECODED? "DEC - export in fully decoded format" :
 settings->parsed==LSTFMT?  "LST - list-format output of traffic table" :
 settings->parsed==PAGEFMT? "PAG - page-full output (for a single aircraft)" : "?"),
(settings->format==TXTFMT?  "TXT - fixed column text output" :
 settings->format==TABFMT?  "TAB - tab-delimited output" :
 settings->format==CSVFMT?  "CSV - comma-separated output" : "?"),
(settings->dstbrg? "distance / bearing" : "latitude / longitude"));
    delay(100);
Serial.printf("\n\
GNS5892 data export options:\n\
    GPIO pin for serial RX from radio module: %d\n\
    GPIO pin for serial TX to radio module: %d\n\
    Receiver comparator level set to %d\n\
    %s\n\
    %s\n\
\nData filtering options:\n\
    range %d-%d nm\n\
    %s\n\
    %s\n\
    %s\n\
    %s\n\
\nDebug verbosity level: %d\n\n",
settings->rx_pin, settings->tx_pin, settings->comparator,
(settings->incl_rssi? "Include RSSI" : "Skip RSSI"),
(settings->chk_crc? "Compute and check CRC" : "Ignore CRC"),
settings->minrange, settings->maxrange,
(settings->alts==LOWALT?  "only show traffic below 18,000 feet" :
 settings->alts==MEDALT?  "only show traffic between 18,000 and 50,000 feet" :
 settings->alts==HIGHALT? "only show traffic above 50,000 feet (shown as 99999)" :
 "ALL - show traffic at all altitudes"),
(settings->dfs==ALLDFS? "show all mode-S & ADS-B downlink frames" :
 settings->dfs==DF17? "show only DF17 (transponder ES) frames" :
 settings->dfs==DF18? "show only DF18 (non-transponder) frames" :
 settings->dfs==DF1718? "show DF17 and DF18 ES downlink frames" :
 settings->dfs==DF20? "show DF4 and DF20 Mode S frames" : "?"),
types, IDs, settings->debug);
    delay(100);
    Serial.println("GNS5892 firmware version below...");
    send5892("#00");
}

static void table()
{
  Serial.printf("\nTRAFFIC TABLE currently:\n\n%d aircraft tracked\n\n", num_tracked);
  const char *s = (settings->format==TXTFMT? " " : settings->format==TABFMT? "\t" : ",");
  char *t = time_string(true);    // a bit later than actual position time
  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
     ufo_t *fop = &container[i];
     // construct a single line of text about each slot in traffic table
     const char *cs = fop->callsign;
     if (cs[0] == '\0' && settings->format == TXTFMT)
        cs = "        ";
     const char *fmt;
     if (fop->addr == 0) {
        // snprintf(parsed, PARSE_BUF_SIZE, "[%2d]\r\n", i);
        parsed[0] = '\0';
     } else if (timenow > fop->positiontime + 15) {
       if (settings->format==TABFMT)
         fmt = "[%d]\t%s\t%d\t%06X\t%s\t%d\r\n";
       if (settings->format==CSVFMT)
         fmt = "[%d],%s,%d,%06X,%s,%d\r\n";
       else // TXTFMT
         fmt = "[%2d] %s %02d %06X %s %02d\r\n";
               //idx time rssi  ID cs actyp
       snprintf(parsed, PARSE_BUF_SIZE, fmt,
         i, t, fop->rssi, fop->addr, cs, fop->aircraft_type);
     } else  if (settings->dstbrg) {
       if (settings->format==TABFMT)
         fmt = "[%d]\t%s\t%d\t%06X\t%s\t%d\t%.1f\t%d\t%s\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n";
       else if (settings->format==CSVFMT)
         fmt = "[%d],%s,%d,%06X,%s,%d,%.1f,%d,%s,%d,%d,%d,%d,%d,%d,%d\r\n";
       else // TXTFMT
         fmt = "[%2d] %s %02d %06X %s %02d %5.1f %3d %s %5d %5d %5d %3d %3d %3d %3d\r\n";
              //idx time rssi ID cs actyp dst brg altitude altdif vs gspd trk aspd hdg
       snprintf(parsed, PARSE_BUF_SIZE, fmt,
         i, t, fop->rssi, fop->addr, cs, fop->aircraft_type,
         (fop->distance==0? 0.1*(float)fop->approx_dist : fop->distance),
         (fop->distance==0? fop->approx_brg  : fop->bearing),
         (fop->alt_type? "g" : ""), fop->altitude, fop->alt_diff, fop->vert_rate,
         fop->groundspeed, fop->track, fop->airspeed, fop->heading);
     } else {
       if (settings->format==TABFMT)
         fmt = "[%d]\t%s\t%d\t%06X\t%s\t%d\t%.4f\t%.4f\t%s\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n";
       else if (settings->format==CSVFMT)
         fmt = "[%d],%s,%d,%06X,%s,%d,%.4f,%.4f,%s,%d,%d,%d,%d,%d,%d,%d\r\n";
       else // TXTFMT
         fmt = "[%2d] %s %02d %06X %s %02d %9.4f %9.4f %s %5d %5d %5d %3d %3d %3d %3d\r\n";
               //idx time rssi ID cs actyp lat lon altitud altdif vs gspd trk aspd hdg
       snprintf(parsed, PARSE_BUF_SIZE, fmt,
         i, t, fop->rssi, fop->addr, cs, fop->aircraft_type,
         fop->latitude, fop->longitude,
         (fop->alt_type? "g" : ""), fop->altitude, fop->alt_diff, fop->vert_rate,
         fop->groundspeed, fop->track, fop->airspeed, fop->heading);
    }
    Serial.print(parsed);
    delay(10);
  }
  Serial.println("");
}

static void stats()
{
    Serial.println("\nSTATISTICS (since last reboot):\n");
    Serial.printf("Input  discards (overflow): %6d\n", in_discards);
    Serial.printf("Output discards (overflow): %6d\n", out_discards);
    int i;
    if (settings->incl_rssi) {
        Serial.println("Messages by RSSI:");
        for (i=0; i<25; i++)
            if (msg_by_rssi[i] > 0)
                Serial.printf("    [%2d] %6d\n", i+22, msg_by_rssi[i]);
    }
    delay(100);
    if (settings->chk_crc) {
        Serial.printf("Messages with CRC error: %d\n", msg_by_crc_cat[1]);
    }
    Serial.println("\nMessages by hour:");
    for (i=0; i<24; i++) {
        if (msg_by_hour[i] > 0)
            Serial.printf("    [%2d] %6d\n", i, msg_by_hour[i]);
    }

    if (settings->parsed == RAWFMT || settings->parsed == RAWFILT || settings->parsed == FLDFMT)
        return;

    delay(100);
    Serial.printf("\nPosition messages with precomputed CPR NL: %6d\n", msg_by_cpr_effort[0]);
    Serial.printf("Position messages with adjacent CPR NL:    %6d\n", msg_by_cpr_effort[1]);
    Serial.printf("Position messages needing CPR NL search:   %6d\n", msg_by_cpr_effort[2]);
    Serial.printf("Position messages with CPR error:          %6d\n", msg_by_cpr_effort[3]);
    Serial.println("\nMessages by DF:");
    for (i=0; i<23; i++) {
        if (msg_by_DF[i] > 0)
            Serial.printf("    [%2d] %6d\n", i, msg_by_DF[i]);
    }
    delay(100);
    Serial.println("\nMessages by altitude category:");
    Serial.printf("    [<18000] %6d\n", msg_by_alt_cat[1]);
    Serial.printf("    [18-50K] %6d\n", msg_by_alt_cat[2]);
    Serial.printf("    [>50K]   %6d\n", msg_by_alt_cat[3]);
    Serial.printf("    [unkn]   %6d\n", msg_by_alt_cat[0]);
    Serial.println("\nMessages by distance category:");
    Serial.printf("    [< 6 nm] %6d\n", msg_by_dst_cat[0]);
    Serial.printf("    [3-30nm] %6d\n", msg_by_dst_cat[1]);
    Serial.printf("    [> 30nm] %6d\n", msg_by_dst_cat[2]);
    delay(100);
    Serial.println("\nMessages by aircraft type:");
    for (i=0; i<16; i++) {
        if (msg_by_aircraft_type[i] > 0)
            Serial.printf("    [%2d] %6d  %s\n", i, msg_by_aircraft_type[i], ac_type_label[i]);
    }
    Serial.println("\nNew aircraft by aircraft type:");
    for (i=0; i<16; i++) {
        if (new_by_aircraft_type[i] > 0)
            Serial.printf("    [%2d] %6d  %s\n", i, new_by_aircraft_type[i], ac_type_label[i]);
    }
    delay(100);
    Serial.println("\nTicks by number tracked:");
    for (i=0; i<MAX_TRACKING_OBJECTS+1; i++) {
        if (ticks_by_numtracked[i] > 0)
            Serial.printf("    [%2d] %6d\n", i, ticks_by_numtracked[i]);
    }
#if defined(TESTING)
    delay(100);
    Serial.printf("    dst approx correct: %6d\n", upd_by_dist_incorrect[0]);
    Serial.printf("               not:     %6d\n", upd_by_dist_incorrect[1]);
    Serial.printf("    brg approx correct: %6d\n", upd_by_brg_incorrect[0]);
    Serial.printf("               not:     %6d\n", upd_by_brg_incorrect[1]);
    Serial.printf("     gs approx correct: %6d\n", upd_by_gs_incorrect[0]);
    Serial.printf("               not:     %6d\n", upd_by_gs_incorrect[1]);
    Serial.printf("    trk approx correct: %6d\n", upd_by_trk_incorrect[0]);
    Serial.printf("               not:     %6d\n", upd_by_trk_incorrect[1]);
    Serial.printf("    ihypotenus correct: %6d\n", ihypot_incorrect[0]);
    Serial.printf("               not:     %6d\n", ihypot_incorrect[1]);
#endif
}

static void help()
{
Serial.println("\
Commands:\n\
RBT - reboot (restores the saved settings)\n\
RST - reset the receiver module\n\
PLP (or simply hit Enter) - play/pause output\n\
TIM,hh:mm - set current time\n\
DAT,yy/mm/dd - set current date (optional, will appear in output)\n\
DBG,n - debug verbosity level n (0-2)\n");

Serial.println("\
Commands that pause the data flow:\n\
HLP - list all commands\n\
COD - list aircraft and message type codes\n\
TBL - list table of aircraft recently seen\n\
STA - show stats (# of frames received, # and % in aircraft types, etc)\n\
SET - show current settings\n\
SAV - save settings to flash memory\n");

Serial.println("\
Settings affecting output format:\n\
NON - do not export detailed data\n\
RAW - export raw hex format\n\
FIL - export raw but filtered\n\
FLD - export in hex format with fields separated\n\
DEC - export in fully decoded format\n\
LST - list format output of traffic table (every 4 seconds)\n\
PAG - page-full output (only for a single aircraft)\n");

Serial.println("\
Field delimiter in output:\n\
TXT - fixed column text output\n\
TAB - tab-delimited output\n\
CSV - comma-separated output\n\
COL - show column titles for output (once, not a setting)\n");

Serial.println("\
Settings that toggle current value:\n\
BRG - show lat/lon or distance/bearing in decoded output\n\
RSS - include RSSI in data (switches 5892 to mode 3+)\n\
BAU - output at 230,400 baud rather than 115,200\n\
CRC - compute and check CRC\n");

Serial.println("\
Data filtering options:\n\
ALL - show all ADS-B traffic (removes filters below)\n\
MIN,dd - only show traffic farther than dd nm ('MIN' for no min)\n\
MAX,dd - only show traffic closer than dd nm ('MAX' for max=180)\n\
LOW - only show traffic below 18,000 feet\n\
MED - only show traffic between 18,000 and 50,000 feet\n\
HIG - only show traffic above 50,000 feet (shown as 99999)\n\
DFS - include all DF types (some in raw export format only)\n\
D17 - only show DF 17 = Extended squitter from transponders\n\
D18 - only show DF 18 = Extended squitter from non-transponders\n\
D78 - only show DF 17 and 18\n\
D20 - only DF 4 and 20 (Mode-S altitude)\n");

Serial.println("\
Settings with parameters:\n\
TYP,tt - only show aircraft type tt (0-15) ('TYP' alone means show all types)\n\
TRK,xxxxxx - show only ICAO ID xxxxxx ('TRK,n by index) ('TRK' alone to cancel)\n\
RIO,nn - GPIO pin for serial RX from radio module <<< REQUIRED\n\
TIO,nn - GPIO pin for serial TX to radio module   <<< REQUIRED\n\
CMP,nn - set the receiver comparator level to nn (10-200, default 100)\n\
LOC,lat,lon - location (decimal degrees, e.g., 42.36,-97.32) <<< REQUIRED\n");
}

static void codes()
{
Serial.println("\
Aircraft types:\
   0 unknown\n\
   1 glider\n\
   2 LTA\n\
   3 parachute\n\
   4 hang glider\n\
   5 reserved\n\
   6 UAV\n\
   7 spacecraft\n\
   8 not used\n\
   9 light\n\
  10 medium 1\n\
  11 medium 2\n\
  12 high vortex\n\
  13 heavy\n\
  14 high perf\n\
  15 rotorcraft\n\
Message type codes:\n\
   I identity (DF17-18)\n\
   P position (DF17-18)\n\
   G position with GNSS altitude\n\
   V velocity (DF17-18)\n\
   L all-call reply (DF11)\n\
   B Comm-B altitude (DF20)\n\
   C ACAS long (DF16)\n\
   S ACAS short (DF 0)\n\
   A mode S altitude (DF 4)\n");
}

static bool setvalue(const char *label, const char *cmd, uint8_t *psetting, uint8_t value, const char *msg)
{
    if (strcmp(label,cmd)==0) {
        pause5892();
        delay(500);
        *psetting = value;
        Serial.print("> ");
        Serial.println(msg);
        delay(1000);
        play5892();
        return true;        // command succeeded
    }
    return false;
}

// interpret the string in the command buffer and execute
void interpret_cmd(char *sentence, int len)
{
  if (len == 0) {     // just crlf
if(settings->debug)
Serial.println("(empty command)");
      play_pause();
      return;
  }
  if (len < 3) {
      help();
      return;
  }

  char *cmd = &sentence[0];
  cmd[3] = '\0';
  strupr(cmd);
  char *param;
  if (len > 4)
      param = &sentence[4];
  else
      param = NULL;

//Serial.print(">>cmd: ");
//Serial.println(cmd);
//if (param == NULL)
//Serial.println("(null param)");

  if (strcmp("RST",cmd)==0) {
      reset5892();
      return;
  }

  if (strcmp("PLP",cmd)==0) {
      play_pause();
      return;
  }

  if (strcmp("COL",cmd)==0) {
      show_columns();
      return;
  }

  // Settings without parameters:
  if (setvalue("NON", cmd, &settings->parsed, NOTHING, "do not export detailed data"))  return;
  if (setvalue("RAW", cmd, &settings->parsed, RAWFMT, "export raw hex format"))  return;
  if (setvalue("FIL", cmd, &settings->parsed, RAWFILT, "export raw but filtered"))  return;
  if (setvalue("FLD", cmd, &settings->parsed, FLDFMT, "export hex format with fields separated"))  return;
  if (setvalue("DEC", cmd, &settings->parsed, DECODED, "export in fully decoded format"))  return;
  if (setvalue("LST", cmd, &settings->parsed, LSTFMT, "list-format output of traffic table"))  return;
  if (setvalue("PAG", cmd, &settings->parsed, PAGEFMT, "page-full output (for a single aircraft)"))  return;

  if (setvalue("DFS", cmd, &settings->dfs, ALLDFS, "show all Mode-S & ADS-B downlink frames"))  return;
  if (setvalue("D17", cmd, &settings->dfs, DF17, "show DF17 (transponder ES) frames"))  return;
  if (setvalue("D18", cmd, &settings->dfs, DF18, "show DF18 (non-transponder ES) frames"))  return;
  if (setvalue("D78", cmd, &settings->dfs, DF1718, "show DF17 and DF18 frames"))  return;
  if (setvalue("D20", cmd, &settings->dfs, DF20, "show Mode-S DF4 and DF20 frames"))  return;

  if (setvalue("TXT", cmd, &settings->format, TXTFMT, "fixed column text output"))  return;
  if (setvalue("TAB", cmd, &settings->format, TABFMT, "tab-delimited output"))  return;
  if (setvalue("CSV", cmd, &settings->format, CSVFMT, "comma-separated output"))  return;

  if (setvalue("LOW", cmd, &settings->alts, LOWALT, "only show traffic below 18,000 feet"))  return;
  if (setvalue("MED", cmd, &settings->alts, MEDALT, "only show traffic between 18,000 and 50,000 feet"))  return;
  if (setvalue("HIG", cmd, &settings->alts, HIGHALT, "only show traffic above 50,000 feet"))  return;

  if (strcmp("ALL",cmd)==0) {
      settings->minrange = 0;
      minrange10 = 0;
      settings->maxrange = 180;
      maxcprdiff = (1<<16);
      maxrange10 = 1800;
      settings->alts = ALLALTS;
      settings->ac_type = 0;
      settings->follow = 0;
      Serial.println("> show all traffic - distances, altitudes, types, ID");
      return;
  }

  // Settings that toggle current value:
  if (settings->incl_rssi)
    {if (setvalue("RSS", cmd, &settings->incl_rssi, 0, "skip RSSI indication in exports"))  return;}
  else
    {if (setvalue("RSS", cmd, &settings->incl_rssi, 1, "include RSSI indication in exports"))  return;}
  if (settings->chk_crc)
    {if (setvalue("CRC", cmd, &settings->chk_crc, 0, "ignore CRC"))  return;}
  else
    {if (setvalue("CRC", cmd, &settings->chk_crc, 1, "compute and check CRC"))  return;}
  if (settings->dstbrg)
    {if (setvalue("BRG", cmd, &settings->dstbrg, 0, "show lat/lon (rather than dst/brg)"))  return;}
  else
    {if (setvalue("BRG", cmd, &settings->dstbrg, 1, "show dst/brg (rather than lat/lon)"))  return;}

  if (strcmp("BAU",cmd)==0) {
    toggle_baud_rate();
    return;
  }

  // Commands that pause the data flow:
  pause5892();
  delay(500);

  // Settings with parameters:

  if (strcmp("TYP",cmd)==0
   || strcmp("TRK",cmd)==0
   || strcmp("MIN",cmd)==0
   || strcmp("MAX",cmd)==0
   || strcmp("DBG",cmd)==0
   || strcmp("RIO",cmd)==0
   || strcmp("TIO",cmd)==0
   || strcmp("CMP",cmd)==0
   || strcmp("LOC",cmd)==0
   || strcmp("TIM",cmd)==0
   || strcmp("DAT",cmd)==0) {

      if (param == NULL) {
          if (strcmp("TYP",cmd)==0) {
              settings->ac_type = 0;
              Serial.println("> show all aircraft types");
          } else if (strcmp("TRK",cmd)==0) {
              settings->follow = 0;
              Serial.println("> show all aircraft IDs");
          } else if (strcmp("MIN",cmd)==0) {
              settings->minrange = 0;
              minrange10 = 0;
              Serial.println("> no minimum distance");
          } else if (strcmp("MAX",cmd)==0) {
              settings->maxrange = 180;
              maxcprdiff = (1<<16);
              maxrange10 = 1800;
              Serial.println("> maximum distance 180nm");
          } else if (strcmp("DBG",cmd)==0) {
              settings->debug = 1;
              Serial.println("> debug level 1");
          } else {
              pause5892();
              help();
          }
          return;
      }
      uint32_t param1 = strtol(param,NULL,0);

      if (strcmp("MIN",cmd)==0) {
          if (param1 >= 0 && param1 <= 100) {
              settings->minrange = param1;
              minrange10 = 10*param1;
              Serial.printf("> minimum distance %d nm\n", param1);
          } else {
              Serial.println("> minimum distance must be between 0 and 100 nm");
          }
          return;
      }

      if (strcmp("MAX",cmd)==0) {
          if (param1 >= 1 && param1 <= 180) {
              settings->maxrange = param1;
              maxcprdiff = (int32_t)((float)(1<<16) * (float)param1 / 180.0);
              maxrange10 = 10*param1;
              Serial.printf("> maximum distance %d nm\n", param1);
          } else {
              Serial.println("> maximum distance must be between 1 and 180 nm");
          }
          return;
      }

      if (strcmp("TYP",cmd)==0) {
          if (param1 >= 0 && param1 <= 15) {
              settings->ac_type = param1;
              if (param1)
                  Serial.printf("> show only aircraft type %d ('TYP' to cancel\n", param1);
              else
                  Serial.println("> show all aircraft types");
          } else {
              pause5892();
              Serial.println("aircraft type must be 0..15");
          }
          return;
      }

      if (strcmp("TRK",cmd)==0) {
          uint32_t addr = strtol(param,NULL,16);
          if (addr < 0x100) {
              if (param1 >= 0 && param1 < MAX_TRACKING_OBJECTS) {
                  addr = container[param1].addr;
              } else {
                  Serial.println("> use TRK,ICAO or TRK,index");
                  return;
              }
          }
          settings->follow = addr;
          Serial.printf("> show only ICAO ID %06X ('TRK' to cancel)\n", addr);
          return;
      }

      if (strcmp("CMP",cmd)==0) {
          if (param1 >= 10 && param1 <= 200) {
              settings->comparator = param1;
              Serial2.printf("#39-00-00-%2X\r", param1);
              Serial.printf("> receiver comparator level set to %d\n", param1);
          } else {
              pause5892();
              Serial.println("> must be between 10 & 200, default 100");
          }
          return;
      }

      if (strcmp("RIO",cmd)==0) {
          if (param1 > 0 && param1 < 40) {
              settings->rx_pin = param1;
              Serial.printf("> RX pin set to %d - now 'SAV' and 'RBT'\n", param1);
          } else {
              Serial.println("> invalid RX pin");
          }
          return;
      }

      if (strcmp("TIO",cmd)==0) {
          if (param1 > 0 && param1 < 34) {
              settings->tx_pin = param1;
              Serial.printf("> TX pin set to %d - now 'SAV' and 'RBT'\n", param1);
          } else {
              Serial.println("> invalid TX pin");
          }
          return;
      }

      if (strcmp("LOC",cmd)==0) {
          sscanf(param, "%f,%f", &settings->latitude, &settings->longitude);
          CPRRelative_precomp();
          Serial.printf("> Our location set to: %f, %f\n", reflat, reflon);
          return;
      }

      if (strcmp("TIM",cmd)==0) {
          sscanf(param, "%d:%d", &ourclock.hour, &ourclock.minute);
          ourclock.second = 0;  // seconds
          Serial.printf("> Our clock set to: %02d:%02d\n", ourclock.hour, ourclock.minute);
          return;
      }

      if (strcmp("DAT",cmd)==0) {
          sscanf(param, "%d/%d/%d", &ourclock.year, &ourclock.month, &ourclock.day);
          Serial.printf("> Our date set to: 20%02d/%02d/%02d\n", ourclock.year, ourclock.month, ourclock.day);
          return;
      }

      if (strcmp("DBG",cmd)==0) {
          if (param1 >= 0 && param1 <= 2) {
              settings->debug = param1;
              Serial.printf("> debug level %d\n", param1);
          } else {
              Serial.println("> debug level must be between 0 and 2");
          }
          return;
      }
              
  }  // end of settings with parameters

  if (strcmp("TBL",cmd)==0) {
      table();
      return;
  }

  if (strcmp("STA",cmd)==0) {
      stats();
      return;
  }

  if (strcmp("SET",cmd)==0) {
      show_settings();
      return;
  }

  if (strcmp("HLP",cmd)==0) {
      help();
      return;
  }

  if (strcmp("COD",cmd)==0) {
      codes();
      return;
  }

  if (strcmp("SAV",cmd)==0) {
      Serial.print("> saving settings to flash...");
      delay(500);
      EEPROM_store();
      delay(1500);
      Serial.println(" done");
      return;
  }

  if (strcmp("RBT",cmd)==0) {
      Serial.println("> rebooting...");
      delay(2000);
      ESP.restart();
      // does not return
  }

  Serial.println("> invalid command (type '?' for list of commands)");
  // help();
}
