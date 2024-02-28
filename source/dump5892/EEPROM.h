/*
 * EEPROM.h
 * Copyright (C) 2024 Moshe Braner
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version - see <http://www.gnu.org/licenses/>.
 */

#ifndef EEPROM_H
#define EEPROM_H

#define EEPROM_MAGIC   0x10905892
#define EEPROM_VERSION 0x00000003

enum
{
    NOTHING,
    RAWFMT,
    RAWFILT,
    FLDFMT,
    DECODED,
    LSTFMT,
    PAGEFMT
};

enum
{
    TXTFMT,
    TABFMT,
    CSVFMT
};

enum
{
    ALLALTS,
    LOWALT,
    MEDALT,
    HIGHALT
};

enum
{
    DFSALL,
    DF1718,
    DF17,
    DF18,
    DF20,
    DFNOTL
};

typedef struct Settings {

    float   latitude;   // of this station (can be approximate)
    float   longitude;  // of this station (can be approximate)
    uint32_t follow;    // ICAO ID to track
    uint8_t  parsed;    // RAWFMT etc
    uint8_t  dstbrg;    // show distance & bearing instead of lat/lon
    uint8_t  minrange;  // nm
    uint8_t  maxrange;  // nm
    uint8_t  alts;      // ALLALTS etc
    uint8_t  format;    // TXTFMT etc
    uint8_t  dfs;       // DF1718 etc
    uint8_t  ac_type;   // to show - if zero then show all
    uint8_t  chk_crc;   // compute CRC & reject if wrong
    uint8_t  incl_rssi; // include RSSI in output
    uint8_t  rx_pin;    // GPIO pin for Serial2 input
    uint8_t  tx_pin;    // GPIO pin for Serial2 output
    uint8_t  comparator;  // GNS5892 module sensitivity setting
    uint8_t  debug;     // debug verbosity level 0,1,2
    uint8_t  outbaud;   // faster output baud rate

    uint8_t  rsvd1;
    uint8_t  rsvd2;
    uint8_t  rsvd3;

} __attribute__((packed)) settings_t;

extern settings_t *settings;
void EEPROM_setup(void);
void EEPROM_store(void);

typedef struct EEPROM_S {
    uint32_t  magic;
    uint32_t  version;
    settings_t settings;
} eeprom_struct_t;

typedef union EEPROM_U {
   eeprom_struct_t field;
   uint8_t raw[sizeof(eeprom_struct_t)];
} eeprom_t;

#endif /* EEPROM_H */
