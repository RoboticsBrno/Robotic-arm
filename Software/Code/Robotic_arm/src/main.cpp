#include <Arduino.h>
#include <iostream>
#include "lx16a.hpp"

using namespace std::chrono_literals; // to use ms/s/h etc.

lw::Bus bus( UART_NUM_1, GPIO_NUM_32 );
lw::Servo s1 = bus.getServo( 3 );
lw::Servo s2 = bus.getServo( 4 );

void setServoId( lw::Id id ) {
    bus.allServos().setId( id );
}

void setup() {
    Serial.begin( 115200 );

    s1.limit( 90_deg, 180_deg );
}


void loop() {
    // Move to position in second
    s1.move( 240_deg, 1000ms );
    s2.move( 240_deg, 1s );
    delay( 3000 );

    // Move to position as fast as possible
    s1.move( 0_deg );
    s2.move( 0_deg );
    delay( 3000 );
}