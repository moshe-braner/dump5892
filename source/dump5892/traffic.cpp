/*
 * traffic.cpp
 * Copyright (C) 2024 Moshe Braner
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version - see <http://www.gnu.org/licenses/>.
 */

#include "dump5892.h"
#include "ApproxMath.h"

// A simple hash table to more quickly find IDs in container[]:
// zero means not present, otherwise *base-1* index into container[].
static uint8_t acindex[256] = {0};

// Info on farthest aircraft, potentially to be replaced with a new closer one
static struct {
    float dist;
    uint32_t addr;
    uint8_t index1;
} farthest = {0, 0, 0};
// Info on closest aircraft
static struct {
    float dist;
    uint32_t addr;
    uint8_t index1;
} closest = {9999.9, 0, 0};

int find_closest_traffic()
{
    if (closest.addr)
        return closest.index1;
    return 0;    // not found
}

int find_traffic_by_addr(uint32_t addr)
{
    int a = (addr & 0x0000FF);
    int i = acindex[a];
    while (i != 0) {
        if (container[i-1].addr == addr)
            return i;
        i = container[i-1].next;
    }
    return 0;    // not found
}

static int find_empty()
{
    int i = find_traffic_by_addr(0);
    if (i == 0)
        num_tracked = MAX_TRACKING_OBJECTS;   // empty slot not found
    return i;
}

// link traffic that is about to be written into container[i-1]
static void insert_traffic_by_index(int i, uint32_t addr)
{
    int k = i-1;
    int a = (addr & 0x0000FF);
    int j = acindex[a];
    acindex[a] = i;
    if (addr == 0) {                 // creating an empty slot
        if (container[k].addr == farthest.addr) {
            farthest.dist = 0;
            farthest.addr = 0;
            farthest.index1 = 0;
        }
        if (container[k].addr == closest.addr) {
            closest.dist = 9999.9;
            closest.addr = 0;
            closest.index1 = 0;
        }
        if (num_tracked > 0)
            --num_tracked;
if(settings->debug>1)
Serial.printf("deleted ID %06X at index0 %d\n", addr, k);
    }
    if (container[k].addr == 0) {    // filling an empty slot
        if (num_tracked < MAX_TRACKING_OBJECTS)
            ++num_tracked;
if(settings->debug>1)
Serial.printf("inserted ID %06X at index0 %d\n", addr, k);
    }
    container[k] = EmptyFO;    // all zeros before writing in new data
    // implies container[i].timestamp = 0;   // until we get a position report
    container[k].addr = addr;
    container[k].next = j;
}

// de-link traffic that is about to be erased from container[i-1]
static void delink_traffic_by_index(int i)
{
    int a = ((container[i-1].addr) & 0x0000FF);
    int j = acindex[a];
    if (j == i) {                           // at head of list
        acindex[a] = container[i-1].next;   // zero if no next
        return;
    }
    while (j != 0) {
        int k = j;
        j = container[j-1].next;
        if (j == i) {
            container[k-1].next = container[i-1].next;
            return;
        }
    }
    // if not found (should not happen) then nothing is done
}

// find existing entry or create a new one
static int add_traffic_by_addr(uint32_t addr, float distance)
{
    // find if already in container[]
    int j = find_traffic_by_addr(addr);
    if (j != 0) {
//if(settings->debug>1)
//Serial.println("add_traffic_by_addr(): already in table");
        return j;
    }

    // else replace an empty object, if any
    j = find_empty();
    if (j != 0) {
        delink_traffic_by_index(j);
        insert_traffic_by_index(j, addr);
if(settings->debug>1)
Serial.println("add_traffic_by_addr(): replaced empty entry in table");
        return (j);
    }

    // else replace farthest (non-followed) object if found
    //   (avoids doing linear search)
    if (distance < farthest.dist) {
        j = farthest.index1;
        farthest.dist = 0;       // will be slowly changed in traffic_update()
        farthest.addr = 0;
        farthest.index1 = 0;
        delink_traffic_by_index(j);
        insert_traffic_by_index(j, addr);
        return (j);
    }

    /* otherwise, no slot found, ignore the new object */
    return 0;
}

// fill in certain fields from each message type
// anything not filled in stays as all zeros

void update_traffic_identity()
{
    // do not create a new entry for an identity message,
    //   wait until position message arrives
    int i = find_traffic_by_addr(fo.addr);
    if (i == 0)
        return;
    ufo_t *fop = &container[i-1];
    int aircraft_type = fo.aircraft_type;
    ++msg_by_aircraft_type[aircraft_type];
    if (fop->aircraft_type == 0)
        ++new_by_aircraft_type[aircraft_type];
    fop->aircraft_type = aircraft_type;
    memcpy(fop->callsign, fo.callsign, 8);
}

void update_traffic_position()
{
    // find in table, or try and create a new entry
    int i = add_traffic_by_addr(fo.addr, fo.distance);
    if (i == 0)
        return;
    ufo_t *fop = &container[i-1];
    if (fop->latitude == 0 && fop->altitude != 0) {
if(settings->debug>1)
Serial.printf("ADS-B overwriting Mode S altitude for ID %06X\n", fo.addr);
    }
    fop->latitude  = fo.latitude;
    fop->longitude = fo.longitude;
    fop->alt_type  = fo.alt_type;
    fop->altitude  = fo.altitude;
    fop->distance  = fo.distance;
    fop->bearing   = fo.bearing;
    if (settings->ac_type != 0) {
        // filtering by aircraft_type, wait until got identity message
        //   - until then, fop->aircraft_type is 0
        fop->positiontime = 0;      // signals do-not-display, filtered out
        if (settings->ac_type == 254) {
            // only show medium & heavy
            if (fop->aircraft_type != 0 && (fop->aircraft_type >= 10 || fop->aircraft_type <= 13))
                fop->positiontime = timenow;
        } else if (settings->ac_type == 255) {
            // exclude medium & heavy
            if (fop->aircraft_type != 0 && (fop->aircraft_type < 10 || fop->aircraft_type > 13))
                fop->positiontime = timenow;
        } else if (fop->aircraft_type == settings->ac_type) {
            fop->positiontime = timenow;
        }
    } else {
        fop->positiontime = timenow;
    }
}

void update_traffic_velocity()
{
    // do not create a new entry until position arrives
    int i = find_traffic_by_addr(fo.addr);
    if (i == 0)
        return;
    ufo_t *fop = &container[i-1];
    fop->ewv = fo.ewv;
    fop->nsv = fo.nsv;
    fop->groundspeed = fo.groundspeed;
    fop->track_is_valid = fo.track_is_valid;
    fop->track = fo.track;
    fop->airspeed_type = fo.airspeed_type;
    fop->airspeed = fo.airspeed;
    fop->heading_is_valid = fo.heading_is_valid;
    fop->heading = fo.heading;
    fop->vert_rate = fo.vert_rate;
    fop->alt_diff = fo.alt_diff;
    fop->velocitytime = timenow;
}

// DF4 Mode S altitude replies - only altitude & ICAO ID
void update_mode_s_traffic()
{
    // find in table, or try and create a new entry
    int i = add_traffic_by_addr(fo.addr, fo.distance);
    if (i == 0)
        return;
    ufo_t *fop = &container[i-1];
    if (fop->latitude == 0) {       // do not overwrite fuller data if available from ADS-B
        //if (fop->altitude == 0) {
        if (fop->altitude != fo.altitude) {
if(settings->debug>1)
Serial.printf("Mode S altitude %d for ID %06X\n", fo.altitude, fo.addr);
        }
        fop->altitude = fo.altitude;
        fop->positiontime = timenow;
    }
}

void traffic_update(int i)
{
    ufo_t *fop = &container[i];
    if (fop->addr == 0)
        return;

    // when should traffic objects expire (as long as there is room)?
    uint32_t exptime = ENTRY_EXPIRATION_TIME;
    if (num_tracked < MAX_TRACKING_OBJECTS)
        exptime <<= 5;
    if (timenow > fop->positiontime + exptime) {
        i++;
        delink_traffic_by_index(i);
        //fop->addr = 0;
        insert_traffic_by_index(i,0);   // link into the empty (0-address) list
        return;
    }

    if (fop->positiontime == 0)   // only ID known, or position filtered out
        return;

    // keep track of which (non-followed) aircraft is farthest
    if (fop->distance > farthest.dist && fop->addr != settings->follow) {
        farthest.dist = fop->distance;
        farthest.addr = fop->addr;
        farthest.index1 = i+1;
    } else if (fop->addr == farthest.addr) {
        if (fop->distance < farthest.dist)
            farthest.dist = fop->distance;   // may not really be the farthest any more
    }
    // keep track of which aircraft is closest
    if (fop->distance > 0 && fop->distance < closest.dist) {
        closest.dist = fop->distance;
        closest.addr = fop->addr;
        closest.index1 = i+1;
    } else if (fop->addr == closest.addr) {
        if (fop->distance > closest.dist)
            closest.dist = fop->distance;   // may not really be the closest any more
    }

#if defined(TESTING)
    if (fop->updatetime < fop->velocitytime) {   // may lag by up to 1 second
        float fgroundspeed = approxHypotenuse( (float)fop->nsv, (float)fop->ewv );
        if ((float)fop->groundspeed > 1.05 * fgroundspeed)
            ++upd_by_gs_incorrect[1];
        else if ((float)fop->groundspeed < 0.95 * fgroundspeed)
            ++upd_by_gs_incorrect[1];
        else
            ++upd_by_gs_incorrect[0];
        float ftrack = atan2_approx((float)fop->nsv, (float)fop->ewv);
        if (ftrack < 0)
            ftrack += 360;
        if (ftrack > 270 && fop->track < 90)
            ftrack -= 360;
        else if (ftrack < 90 && fop->track > 270)
            ftrack += 360;
        if (fop->groundspeed>0 && fabs(ftrack-fop->track) > 3)
            ++upd_by_trk_incorrect[1];
        else
            ++upd_by_trk_incorrect[0];
    }
#endif

#if defined(TESTING)
    if (fop->updatetime < fop->positiontime) {   // may lag by up to 1 second
        float x, y;
        y = (111300.0 * 0.00053996) * (fop->latitude - reflat); /* nm */
        x = (111300.0 * 0.00053996) * (fop->longitude - reflon) * CosLat(reflat);
        float fdistance = approxHypotenuse(x, y);
        if (fop->distance > 1.02 * fdistance)
            ++upd_by_dist_incorrect[1];
        else if (fop->distance < 0.98 * fdistance)
            ++upd_by_dist_incorrect[1];
        else
            ++upd_by_dist_incorrect[0];
        int16_t fbearing = (int16_t) atan2_approx(y, x);     /* degrees from ref to target */
        if (fbearing < 0)
            fbearing += 360;
        if (abs(fop->bearing - fbearing) > 2)
            ++upd_by_brg_incorrect[1];
        else
            ++upd_by_brg_incorrect[0];
    }
#endif

    fop->updatetime = timenow;
}

void traffic_setup()
{
    // link all the empty slots off of acindex[0]
    // so that find_traffic_by_addr(0) will find them
    acindex[0] = 1;   // pointing to container[0]
    for (int i=0; i<MAX_TRACKING_OBJECTS-1; i++)
        container[i].next = i+2;
    container[MAX_TRACKING_OBJECTS-1].next = 0;

    //num_tracked = 0;
    //farthest.dist = 0;
    //farthest.addr = 0;
    //farthest.index1 = 0;
    //closest.dist = 9999.9;
    //closest.addr = 0;
    //closest.index1 = 0;
}

void traffic_loop()
{
    // update some things (for all aircraft, one at a time) periodically
    static unsigned int tick = 0;
    static uint32_t nexttime = 0;
    if (millis() < nexttime)
        return;
    nexttime = millis() + (3000/MAX_TRACKING_OBJECTS);   // each one every 3 seconds
    tick++;

    // choose which entry to update this time around the loop
    // assumes MAX_TRACKING_OBJECTS is a power of 2
    int i = (tick & (MAX_TRACKING_OBJECTS-1));
    if (i >= MAX_TRACKING_OBJECTS)
        i = 0;                        // just to be safe

    traffic_update(i);

    ++ticks_by_numtracked[num_tracked];
}
