/*
 * globals.cpp
 * Copyright (C) 2024 Moshe Braner
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version - see <http://www.gnu.org/licenses/>.
 */

#include "dump5892.h"

const char *hex = "0123456789ABCDEF";

char buf[256];      // input from the GNS5892
int inputchars;
unsigned char msg[14];
char parsed[PARSE_BUF_SIZE];   // enough room for the "page" style output
int parsedchars;
bool parsing_success;
bool paused;
char cmdbuf[128];    // commands from console
uint8_t cmdchars = 0;

uint16_t minrange10 = 0;
uint16_t maxrange10 = 1800;   // tenths of nm (setting is in whole nm)

// our position
float reflat, reflon;

// our clock
uint32_t timenow;  // seconds
ourclock_t ourclock;

// variables filled in by message parsing
mm_t mm, EmptyMsg;

// variables precomputed for decoding of CPR lat/lon, based on our own location
int NL[2];
float dLat[2], flrlat[2], modlat[2], dLon[2], flrlon[2], modlon[2];
uint32_t ourcprlat[2], ourcprlon[2];
float dLatHalf, dLonHalf;
int32_t maxcprdiff, maxcprdiff_sq;

// similar values precomputed for adjacent NL zones
int32_t cprMinuslat[2], cprNL0lat[2], cprNL1lat[2], cprPluslat[2];
float dLonPlus[2], flrlonPlus[2], modlonPlus[2];
float dLonMinus[2], flrlonMinus[2], modlonMinus[2];
uint32_t ourcprlonPlus[2], ourcprlonMinus[2];

// the structures holding aircraft data
ufo_t container[MAX_TRACKING_OBJECTS];
ufo_t EmptyFO = {0};
ufo_t fo;
int num_tracked = 0;

const char* ac_type_label[16] PROGMEM = {
    "unknown",
    "glider",
    "LTA",
    "parachute",
    "hang glider",
    "reserved",
    "UAV",
    "spacecraft",
    "not used",
    "light",
    "medium 1",
    "medium 2",
    "high vortex",
    "heavy",
    "high perf",
    "rotorcraft"
};

// stored statistics
int msg_by_rssi[25];
int msg_by_crc_cat[2];
int msg_by_cpr_effort[4];
int msg_by_DF[23];
int msg_by_alt_cat[4];
int msg_by_dst_cat[4];
int msg_by_aircraft_type[16];
int new_by_aircraft_type[16];
int msg_by_hour[24];
int ticks_by_numtracked[MAX_TRACKING_OBJECTS+1];
int in_discards;
int out_discards;
#if defined(TESTING)
int upd_by_gs_incorrect[2];
int upd_by_trk_incorrect[2];
int upd_by_dist_incorrect[2];
int upd_by_brg_incorrect[2];
#endif
