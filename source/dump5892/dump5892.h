/*
 * dump5892.h
 * Copyright (C) 2024 Moshe Braner
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version - see <http://www.gnu.org/licenses/>.
 */

#ifndef DUMP5892_H
#define DUMP5892_H

#if defined(ARDUINO)
#include <Arduino.h>
#endif

#include "EEPROM.h"

#define FIRMWARE_VERSION "v09"
#define FIRMWARE_IDENT   "dump5892"

#define TESTING

#define ENTRY_EXPIRATION_TIME  30 /* seconds */

#define SERIAL_IN_BR      921600
#define SERIAL_OUT_BR     115200
#define HIGHER_OUT_BR     230400

#define INPUT_BUF_SIZE      1000
#define OUTPUT_BUF_SIZE     1000

#define PARSE_BUF_SIZE      1000

#define MAX_TRACKING_OBJECTS 32   // must be a power of 2

typedef struct UFO {
    uint32_t  addr;
    uint32_t  positiontime;
    uint32_t  velocitytime;
    uint32_t  updatetime;
    uint32_t  reporttime;
    float    latitude;
    float    longitude;
    uint32_t  altitude;
    float    distance;
//    uint16_t  approx_dist;   // tenths-of-nm
    int16_t   bearing;
//    int16_t   approx_brg;
    uint8_t   alt_type;
    uint8_t   aircraft_type;
    int16_t   ewv;
    int16_t   nsv;
    int16_t   groundspeed;
    int16_t   track;
    uint8_t   track_is_valid;
    uint8_t   heading_is_valid;
    int16_t   heading;
    int16_t   airspeed;
    uint8_t   airspeed_type;
    uint8_t   vert_rate_source;
    int16_t   vert_rate;
    int16_t   alt_diff;
    char callsign[10];
    uint8_t   rssi;
    uint8_t   next;
} ufo_t;

extern const char* ac_type_label[16];

extern const char *hex;

extern char buf[];
extern int inputchars;
extern byte msg[14];
extern char parsed[];
extern int parsedchars;
extern bool parsing_success;
extern bool paused;
extern char cmdbuf[];
extern uint8_t cmdchars;
extern int bytes_per_ms;

extern uint16_t minrange10;
extern uint16_t maxrange10;

// our position
extern float reflat, reflon;

// our clock
extern uint32_t timenow;
typedef struct aclock {
    uint32_t nextsecond;   // millis() when second will be incremented
    int second;
    int minute;
    int hour;
    int day;
    int month;
    int year;
} ourclock_t;
extern ourclock_t ourclock;

// variables filled in by message parsing
typedef struct mmstruct {
    int frame;     // DF
    int type;      // TC
    int sub;       // subtype
    char msgtype;  // I, P or V
    int fflag;     // odd/even
    uint32_t cprlat;    // 17-bit relative representation
    uint32_t cprlon;
} mm_t;
extern mm_t mm, EmptyMsg;

// variables precomputed for decoding of CPR lat/lon
extern int NL[2];
extern float dLat[2], flrlat[2], modlat[2], dLon[2], flrlon[2], modlon[2];
extern int32_t cprMinuslat[2], cprNL0lat[2], cprNL1lat[2], cprPluslat[2];
extern uint32_t ourcprlat[2], ourcprlon[2];
extern float dLatHalf, dLonHalf;
extern int32_t maxcprdiff, maxcprdiff_sq;
extern float dLonPlus[2], flrlonPlus[2], modlonPlus[2];
extern float dLonMinus[2], flrlonMinus[2], modlonMinus[2];
extern uint32_t ourcprlonPlus[2], ourcprlonMinus[2];

// the structures holding aircraft data
extern ufo_t container[MAX_TRACKING_OBJECTS];
extern ufo_t fo;
extern ufo_t EmptyFO;
extern int num_tracked;

// stored statistics
extern int msg_by_rssi[25];
extern int msg_by_crc_cat[2];
extern int msg_by_cpr_effort[4];
extern int msg_by_DF[23];
extern int msg_by_type[26];
extern int gray_count[4];
extern int msg_by_alt_cat[4];
extern int msg_by_dst_cat[4];
extern int msg_by_aircraft_type[16];
extern int new_by_aircraft_type[16];
extern int msg_by_hour[24];
extern int ticks_by_numtracked[MAX_TRACKING_OBJECTS+1];
extern int in_discards;
extern int out_discards;
#if defined(TESTING)
extern int upd_by_gs_incorrect[2];
extern int upd_by_trk_incorrect[2];
extern int upd_by_dist_incorrect[2];
extern int upd_by_brg_incorrect[2];
#endif

void reset5892();
void pause5892();
void play5892();
void show_settings();
void interpret_cmd(char *sentence, int len);
bool parse(char *buf, int n);
int find_traffic_by_addr(uint32_t addr);
int find_closest_traffic();
void update_traffic_identity();
void update_traffic_position();
void update_traffic_velocity();
void update_mode_s_traffic();
void traffic_update(int i);
void traffic_setup();
void traffic_loop();
int decodeCPRrelative();
void CPRRelative_precomp();
void CPRRelative_setup();
uint32_t check_crc( int n );
void EEPROM_setup();
void EEPROM_store();
char *time_string(bool withdate);

#endif  // DUMP5892_H
