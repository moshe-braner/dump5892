/*
 * EEPROM.cpp
 * Copyright (C) 2024 Moshe Braner
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version - see <http://www.gnu.org/licenses/>.
 */

#include <EEPROM.h>

#include "dump5892.h"

eeprom_t eeprom_block;
settings_t *settings;

static void EEPROM_defaults()
{
    eeprom_block.field.magic   = EEPROM_MAGIC;
    eeprom_block.field.version = EEPROM_VERSION;

    settings->latitude=0;
    settings->longitude=0;
    settings->follow=0;    // ICAO ID to track
    settings->parsed = RAWFMT;
    settings->format = TXTFMT;
    settings->dstbrg = 0;
    settings->minrange = 0;
    settings->maxrange = 180;
    settings->alts = ALLALTS;
    settings->dfs = DF1718;
    settings->ac_type = 0;
    settings->chk_crc = 0;
    settings->incl_rssi = 0;
    settings->rx_pin = 255;  // 16;
    settings->tx_pin = 255;  // 17;
    settings->comparator = 100;
    settings->outbaud = 0;
    settings->debug = 0;
}


void EEPROM_setup()
{

  if (! EEPROM.begin(sizeof(eeprom_t)))
  {
    Serial.print(F("Failed to initialize "));
    Serial.print(sizeof(eeprom_t));
    Serial.println(F(" bytes of EEPROM!"));
    Serial.flush();
    delay(1000000);
  }

  for (int i=0; i<sizeof(eeprom_t); i++) {
    eeprom_block.raw[i] = EEPROM.read(i);
  }

  settings = &eeprom_block.field.settings;

  if (eeprom_block.field.magic != EEPROM_MAGIC) {
    Serial.println(F("User defined settings are not initialized yet. Loading defaults..."));
    EEPROM_defaults();
  } else {
    Serial.print(F("EEPROM version: "));
    Serial.println(eeprom_block.field.version);
    if (eeprom_block.field.version != EEPROM_VERSION) {
      Serial.println(F("WARNING! Version mismatch of user defined settings. Loading defaults..."));
      EEPROM_defaults();
    }
  }
}

void EEPROM_store()
{
  //Serial.println("Writing EEPROM...");
  for (int i=0; i<sizeof(eeprom_t); i++) {
    EEPROM.write(i, eeprom_block.raw[i]);
  }
  EEPROM.commit();
}
