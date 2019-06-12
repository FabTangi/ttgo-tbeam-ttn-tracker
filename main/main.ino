/*

Main module

# Modified by Kyle T. Gabriel to fix issue with incorrect GPS data for TTNMapper

Copyright (C) 2018 by Xose Pérez <xose dot perez at gmail dot com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "configuration.h"
#include <rom/rtc.h>
#include "t_beam.h"

uint8_t txBuffer[11];
uint8_t tGPS[9];
TinyGPSLocation mylocation;
T_beam t_beam = T_beam(); 

uint32_t LatitudeBinary, LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;
uint16_t vbat;
char t[32]; // used to sprintf for Serial output

// Message counter, stored in RTC memory, survives deep sleep
RTC_DATA_ATTR uint32_t count = 0;

// -----------------------------------------------------------------------------
// Application
// -----------------------------------------------------------------------------

void send() {
    char buffer[40];
    Serial.print("sending");
    snprintf(buffer, sizeof(buffer), "Latitude: \n", mylocation.lat());
    screen_print(buffer);
    snprintf(buffer, sizeof(buffer), "Longitude: %10.6f\n", mylocation.lng());
    screen_print(buffer);
    snprintf(buffer, sizeof(buffer), "Error: %4.2fm\n", 1); //hdop
    screen_print(buffer);

    buildPacket(mylocation.lat(),mylocation.lng());

    #if LORAWAN_CONFIRMED_EVERY > 0
        bool confirmed = (count % LORAWAN_CONFIRMED_EVERY == 0);
    #else
        bool confirmed = false;
    #endif

    ttn_cnt(count);
    ttn_send(txBuffer, sizeof(txBuffer), LORAWAN_PORT, confirmed);

    count++;
    sleep();
}


void buildPacket(float lat, float lng)
{
  LatitudeBinary = ((lat + 90) / 180.0) * 16777215;
  LongitudeBinary = ((lng + 180) / 360.0) * 16777215;
  
  sprintf(t, "Lat: %f", lat);
  Serial.println(t);
  
  sprintf(t, "Lng: %f", lng);
  Serial.println(t);

  vbat = (float)(analogRead(BAT_PIN)) / 4095*2*3.3*1.1*100 ;
  Serial.println(String(vbat));
  
  txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
  txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
  txBuffer[2] = LatitudeBinary & 0xFF;

  txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
  txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
  txBuffer[5] = LongitudeBinary & 0xFF;

  altitudeGps = 100;
  txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
  txBuffer[7] = altitudeGps & 0xFF;

  hdopGps = 10/10;
  txBuffer[8] = hdopGps & 0xFF;

  txBuffer[9] = ((vbat*100)>> 8 ) & 0xFF;
  txBuffer[10] = (vbat*100) & 0xFF;
  
}


void sleep() {
    #if SLEEP_BETWEEN_MESSAGES
        Serial.println("\n Sleep function");
        // Show the going to sleep message on the screen
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Sleeping in %3.1fs\n", (MESSAGE_TO_SLEEP_DELAY / 1000.0));
        screen_print(buffer);

        // Wait for MESSAGE_TO_SLEEP_DELAY millis to sleep
        delay(MESSAGE_TO_SLEEP_DELAY);

        // Turn off screen
        screen_off();

        // Set the user button to wake the board
        sleep_interrupt(BUTTON_PIN, LOW);
        Serial.println("\n GPS Sleep");    
        t_beam.low_power_deep_sleep_timer();
        //sleep_millis(sleep_for);
        
    #endif
}

void callback(uint8_t message) {
    if (EV_JOINING == message) screen_print("Joining TTN...\n");
    if (EV_JOINED == message) screen_print("TTN joined!\n");
    if (EV_JOIN_FAILED == message) screen_print("TTN join failed\n");
    if (EV_REJOIN_FAILED == message) screen_print("TTN rejoin failed\n");
    if (EV_RESET == message) screen_print("Reset TTN connection\n");
    if (EV_LINK_DEAD == message) screen_print("TTN link dead\n");
    if (EV_ACK == message) screen_print("ACK received\n");
    if (EV_PENDING == message) screen_print("Message discarded\n");
    if (EV_QUEUED == message) screen_print("Message queued\n");

    if (EV_TXCOMPLETE == message) {
        screen_print("Message sent\n");
        sleep();
    }

    if (EV_RESPONSE == message) {

        screen_print("[TTN] Response: ");

        size_t len = ttn_response_len();
        uint8_t data[len];
        ttn_response(data, len);

        char buffer[6];
        for (uint8_t i=0; i<len; i++) {
            snprintf(buffer, sizeof(buffer), "%02X", data[i]);
            screen_print(buffer);
        }
        screen_print("\n");
    }
}

uint32_t get_count() {
    return count;
}

void setup() {
    // Debug
    #ifdef DEBUG_PORT
        DEBUG_PORT.begin(SERIAL_BAUD);
    #endif

    // Buttons & LED
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BAT_PIN, INPUT);

    // Hello
    DEBUG_MSG(APP_NAME " " APP_VERSION "\n");

    // Display
    screen_setup();

    // Show logo on first boot
    if (0 == count) {
        screen_print(APP_NAME " " APP_VERSION, 0, 0);
        screen_show_logo();
        screen_update();
        delay(LOGO_DELAY);
    }

    // TTN setup
    DEBUG_MSG("\nSetup TTN\n");
    if (!ttn_setup()) {
        screen_print("[ERR] Radio module not found!\n");
        delay(MESSAGE_TO_SLEEP_DELAY);
        screen_off();
        sleep_forever();
    }
    DEBUG_MSG("TTN2\n");
    ttn_register(callback);
    DEBUG_MSG("TTN3\n");
    ttn_join();
    DEBUG_MSG("TTN4\n");
    ttn_sf(LORAWAN_SF);
    ttn_adr(LORAWAN_ADR);
}

void loop() { 
    
    //gps_loop();
    DEBUG_MSG("LOOP_BEGIN\n");    
    //mylocation = t_beam.getLocation();
    ttn_loop();
    screen_loop();    
    // Send every SEND_INTERVAL millis
    static uint32_t last = 0;
    static bool first = true;
    Serial.print(last);
    Serial.print(millis() - last);
    if (0 == last || millis() - last > SEND_INTERVAL) {
        mylocation = t_beam.getLocation();
        Serial.print(F("SUCCES\n"));

        //Serial.println(gps_hdop());
        //Serial.println(gps_latitude());
        //Serial.println(gps_longitude());
        if (mylocation.lat() != 0 && mylocation.lng() != 0) {
            DEBUG_MSG("LOOP_SEND\n");
            last = millis();
            first = false;
            send();         
        } else {   
            if (first) {
                DEBUG_MSG("Waiting GPS\n");
                screen_print("Waiting GPS lock\n");
                first = false;
            }
            if (millis() > GPS_WAIT_FOR_LOCK) {                
                DEBUG_MSG("Sleep\n"); 
                sleep();
            }
        }
    }
}
