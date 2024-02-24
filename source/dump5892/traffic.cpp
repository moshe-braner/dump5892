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

// Info on farthest aircraft, potentially to be replaced with closer
static struct {
    uint32_t addr;
    uint16_t dist;
    uint8_t index1;
} farthest = {0};

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
static int add_traffic_by_addr(uint32_t addr, uint16_t approx_dist)
{
    // find if already in container[]
    int j = find_traffic_by_addr(addr);
    if (j != 0) {
if(settings->debug>1)
Serial.println("add_traffic_by_addr(): already in table");
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
    if (approx_dist < farthest.dist) {
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
    // do not create a new entry until position arrives
    int i = find_traffic_by_addr(fo.addr);
    if (i == 0)
        return;
    ufo_t *fop = &container[i-1];
    int aircraft_type = fo.aircraft_type;
    ++msg_by_aircraft_type[aircraft_type];
    fop->aircraft_type = aircraft_type;
    memcpy(fop->callsign, fo.callsign, 8);
}

void update_traffic_position()
{
    // find in table, or try and create a new entry
    int i = add_traffic_by_addr(fo.addr, fo.approx_dist);
    if (i == 0)
        return;
    ufo_t *fop = &container[i-1];
    fop->latitude = fo.latitude;
    fop->longitude = fo.longitude;
    fop->alt_type = fo.alt_type;
    fop->altitude = fo.altitude;
    fop->approx_dist = fo.approx_dist;
    fop->approx_brg = fo.approx_brg;
    fop->positiontime = timenow;
    if (fop->callsign[0] != 0) {        // got identity message, so have aircraft_type
        if (settings->ac_type != 0 && fop->aircraft_type != settings->ac_type)
            fop->positiontime = 0;      // signals do-not-display, filtered out
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
  //fop->groundspeed = fo.groundspeed;
  //fop->track_is_valid = fo.track_is_valid;
  //fop->track = fo.track;
    // actually get either groundspeed+track or airspeed+heading, not both
    // in the case of groundspeed, got it as ewv, nsv
    // compute groundspeed in traffic_update()
    fop->airspeed_type = fo.airspeed_type;
    fop->airspeed = fo.airspeed;
    fop->heading_is_valid = fo.heading_is_valid;
    fop->heading = fo.heading;
    fop->vert_rate = fo.vert_rate;
    fop->alt_diff = fo.alt_diff;
    fop->velocitytime = timenow;
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
    if (fop->approx_dist > farthest.dist && fop->addr != settings->follow) {
        farthest.dist = fop->approx_dist;
        farthest.addr = fop->addr;
        farthest.index1 = i+1;
    } else if (fop->addr == farthest.addr) {
        if (fop->approx_dist < farthest.dist)
            farthest.dist = fop->approx_dist;   // may not really be the farthest any more
    }

    // fill in the fields that require relatively expensive "math":
    if (fop->updatetime < fop->velocitytime) {   // may lag by up to 1 second
        // Compute velocity and angle from the two speed components
        fop->groundspeed = iapproxHypotenuse0(fop->nsv, fop->ewv);
        if (fop->groundspeed > 0) {
            fop->track = iatan2_approx(fop->nsv, fop->ewv);
            // We don't want negative values but a 0-360 scale.
            if (fop->track < 0)
                fop->track += 360;
            fop->track_is_valid = 1;
#if defined(TESTING)
        float fgroundspeed = approxHypotenuse( (float)fop->nsv, (float)fop->ewv );
        if ((float)fop->groundspeed > 1.05 * fgroundspeed)
            ++upd_by_gs_incorrect[1];
        else if ((float)fop->groundspeed < 0.95 * fgroundspeed)
            ++upd_by_gs_incorrect[1];
        else
            ++upd_by_gs_incorrect[0];
        float ftrack = atan2_approx((float)fop->nsv, (float)fop->ewv);
        if (ftrack > 270 && fop->track < 90)
            ftrack -= 360;
        else if (ftrack < 90 && fop->track > 270)
            ftrack += 360;
        if (fabs(ftrack - fop->track) > 3)
            ++upd_by_trk_incorrect[1];
        else
            ++upd_by_trk_incorrect[0];
#endif
        } else if (fop->airspeed > 0) {
            // if groundspeed is not available use airspeed
            fop->groundspeed = fop->airspeed;
            fop->track = fop->heading;
            fop->track_is_valid = 1;
        } else {
          fop->track = 0;
          fop->track_is_valid = 0;
        }
    }

    // if followed, also compute distance & bearing
#if defined(TESTING)
    if (true) {
#else
    if (fop->addr == settings->follow) {
#endif
        if (fop->updatetime < fop->positiontime) {   // may lag by up to 1 second
            float x, y;
            y = (111300.0 * 0.00053996) * (fop->latitude - reflat); /* nm */
            x = (111300.0 * 0.00053996) * (fop->longitude - reflon) * CosLat(reflat);
            fop->distance = approxHypotenuse(x, y);
#if defined(TESTING)
            if (fop->distance > 1.05 * 0.1 * (float)fop->approx_dist)
                ++upd_by_dist_incorrect[1];
            else if (fop->distance < 0.95 * 0.1 * (float)fop->approx_dist)
                ++upd_by_dist_incorrect[1];
            else
                ++upd_by_dist_incorrect[0];
            float idist = 0.001*(float)iapproxHypotenuse1((int32_t)(1000*x), (int32_t)(1000*y));
            if (fop->distance > 1.01 * idist)
                ++ihypot_incorrect[1];
            else if (fop->distance < 0.99 * idist)
                ++ihypot_incorrect[1];
            else
                ++ihypot_incorrect[0];
#endif
            int16_t ib = (int16_t) atan2_approx(y, x);     /* degrees from ref to target */
            if (ib < 0)
                ib += 360;
            fop->bearing = ib;
#if defined(TESTING)
//Serial.printf("traffic_update(): approx dst/brg: %d %d\n", fop->approx_dist, fop->approx_brg);
//Serial.printf("traffic_update(): comput dst/brg: %.1f %d\n", fop->distance, fop->bearing);
            // and/or use iatan2_approx(cprlatdiff,cprlondiff) in parse
            if (abs(fop->bearing - fop->approx_brg) > 3)
                ++upd_by_brg_incorrect[1];
            else
                ++upd_by_brg_incorrect[0];
#endif
        }
    }

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
}

void traffic_loop()
{
    // update groundspeed, track (for all aircraft, one at a time) periodically
    static unsigned int tick = 0;
    static uint32_t nexttime = 0;
    if (millis() < nexttime)
        return;
    nexttime = millis() + (2000/MAX_TRACKING_OBJECTS);   // each one every 2 seconds
    tick++;

    // choose which entry to update
    // assumes MAX_TRACKING_OBJECTS is a power of 2
    int i = (tick & (MAX_TRACKING_OBJECTS-1));
    if (i >= MAX_TRACKING_OBJECTS)
        i = 0;                        // just to be safe

    traffic_update(i);

    ++ticks_by_numtracked[num_tracked];
}
