/*

Credentials file

*/

#pragma once

// Only one of these settings must be defined
#define USE_ABP
//#define USE_OTAA

#ifdef USE_ABP

    // LoRaWAN NwkSKey, network session key MSB format
    static const u1_t PROGMEM NWKSKEY[16] = { 0xCB, 0x8E, 0xA9, 0x90, 0x73, 0x58, 0x8E, 0xDC, 0xE9, 0x41, 0xF4, 0x59, 0x57, 0xD6, 0x97, 0x45 };
    // LoRaWAN AppSKey, application session key MSB format
    static const u1_t PROGMEM APPSKEY[16] = { 0x56, 0x0C, 0xC6, 0xA6, 0xDD, 0x0F, 0x3F, 0xA9, 0xE6, 0xED, 0xAA, 0x6D, 0x88, 0x09, 0x19, 0xE5 };
    // LoRaWAN end-device address (DevAddr)
    // This has to be unique for every node
    static const u4_t DEVADDR = 0x26011FF6;

#endif

#ifdef USE_OTAA

    // This EUI must be in little-endian format, so least-significant-byte
    // first. When copying an EUI from ttnctl output, this means to reverse
    // the bytes. For TTN issued EUIs the last bytes should be 0x00, 0x00,
    // 0x00.
    static const u1_t PROGMEM APPEUI[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    // This should also be in little endian format, see above.
    static const u1_t PROGMEM DEVEUI[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    // This key should be in big endian format (or, since it is not really a
    // number but a block of memory, endianness does not really apply). In
    // practice, a key taken from ttnctl can be copied as-is.
    // The key shown here is the semtech default key.
    static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

#endif
