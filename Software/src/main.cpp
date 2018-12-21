#include <Arduino.h>
#include <iostream>
#include "lx16a.hpp"

using namespace std::chrono_literals; // to use ms/s/h etc.

lw::Bus bus( UART_NUM_1, GPIO_NUM_32 );
lw::Servo s1 = bus.getServo( 1 );
lw::Servo s2 = bus.getServo( 2 );
lw::Servo s3 = bus.getServo( 3 );

void setServoId( lw::Id id ) {
    bus.allServos().setId( id );
}

void setup() {
    Serial.begin( 115200 );

    //setServoId(1);    

    s1.limit( 0_deg, 180_deg );
    s2.limit( 6_deg, 240_deg );
    s3.limit( 0_deg, 240_deg );

    

}


void loop() {
    // Move to position in second
    //s1.move( 0_deg, 1000ms );
    //s2.move( 240_deg, 1s );
    
    
    // Move to position as fast as possible
    s1.move( 180_deg ,2s);
    s2.move( 7_deg ,2s);
    //s3.move( 0_deg );
    delay( 6000 );

    s2.move( 96_deg ,2s);
    delay( 3000 );

    s1.move( 90_deg ,2s);
    delay( 3000 );

    s1.move( 180_deg ,2s);
    s2.move( 7_deg ,2s);
    delay( 3000 );

    s1.move( 20_deg ,2s);
    s2.move( 160_deg ,2s);
    delay( 3000 );


}