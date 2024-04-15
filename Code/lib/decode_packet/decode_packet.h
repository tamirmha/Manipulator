#ifndef _decode_packet_h
#define _decode_packet_h
#include "Arduino.h" 
#include "servo.h"

//! This module decodes packets in this format: "[char][arg1],[arg2]\n"

void split_packet(String packet, String *arg1, String *arg2)
{
    int index = 0, prev_index = 1;

    index = packet.indexOf(',', prev_index);
    if(index>0) //if found ","
    {
        *arg1 = packet.substring(prev_index, index);
        prev_index = index + 1;

        index = packet.indexOf('\n', prev_index);
        *arg2 = packet.substring(prev_index, index);
        prev_index = index + 1;
    }
    else  //no "," now check if theres \n and it means arg2 is empty
    {
        index = packet.indexOf('\n', prev_index);
        *arg1 = packet.substring(prev_index, index);
        prev_index = index + 1;
    }

    // Serial.println("arg1 = " + arg1);
    // Serial.println("arg2 = " + arg2);
}

/**
 * @brief Function to decode incoming data from any sender (uart / server / whatever)
 * @param packet the packet that needs to be decoded
 * @param sender sender of the packet
 * @return void
 */
void decode_packet(String packet, int sender)
{
    Serial.printf("packet from: %d, received: ", sender);
    Serial.println(packet);

    String arg1 = "", arg2 = "";
    split_packet(packet, &arg1, &arg2);

    String response = "";
    
    
    switch(packet[0])
    {
        case '-':
            break;

        case 'a':
        {
            int angle = arg1.toInt();
            Serial.println(angle);
            servo_control(angle, 6);
            servo_control(angle, 0);
            servo_control(angle, 2);
            break;
        }
        
        default:
            Serial.println("unknown command");
            break;
    }

}

void serial_listener()
{
  if(Serial.available())
  {
    String st = Serial.readStringUntil('\n');
    decode_packet(st, 100);
  }
}
#endif