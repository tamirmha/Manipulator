#ifndef _encoder_h
#define _encoder_h

#include <Arduino.h>

struct EncoderPins
{
    int Neck = 12;
    int Shoulder = 13;
    int Elbow = 14;
    int Wrist1 = 27;
    int Wrist2 = 26;
    int Wrist3 = 15;  //25
    int Gripper = 33;
};

int angle_from_encoder(int encoder_pin)
{
    int enc_value = analogRead(encoder_pin);
    int angle = int(enc_value/22.75);
    return angle;
}

void encoder_setup()
{
    EncoderPins pins;
    pinMode(pins.Neck, INPUT);
    pinMode(pins.Shoulder, INPUT);
    pinMode(pins.Elbow, INPUT);
    pinMode(pins.Wrist1, INPUT);
    pinMode(pins.Wrist2, INPUT);
    pinMode(pins.Wrist3, INPUT);
    pinMode(pins.Gripper, INPUT);
}

std::array<int, NUM_MOTORS> encoder_loop()
{
    EncoderPins pins;
    return {angle_from_encoder(pins.Neck), 
            angle_from_encoder(pins.Shoulder), 
            angle_from_encoder(pins.Elbow), 
            angle_from_encoder(pins.Wrist1), 
            angle_from_encoder(pins.Wrist2),
            angle_from_encoder(pins.Wrist3),
            angle_from_encoder(pins.Gripper)};
}
#endif