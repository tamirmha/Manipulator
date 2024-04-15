#include <Arduino.h>
#include <decode_packet.h>
#include <encoder.h>
#include <my_remotexy.h>


void setup() {
    Serial.begin(115200);
    setup_servo();
    encoder_setup();
    remotexy_setup();
}


void loop() {
    serial_listener();
    remotexy_loop();
    if (RemoteXY.connect_flag)
    {   
      servo_control(remotexy2angle(RemoteXY.Base), 0);
      servo_control(remotexy2angle(RemoteXY.Shoulder), 1);
      servo_control(remotexy2angle(RemoteXY.wrist1), 2);
      servo_control(remotexy2angle(RemoteXY.wrist2), 3);
      servo_control(remotexy2angle(RemoteXY.wrist3), 4);
      servo_control(remotexy2angle(RemoteXY.Base), 5);
      if (RemoteXY.Gripper)
        servo_control(0, 6);
      else
        servo_control(SERVO_MAX_DEGREE, 6);
    }
    else  // Encoders control the motors
    {
      std::array<int, NUM_MOTORS> angles = encoder_loop();
      for(int i=0; i < NUM_MOTORS; i++)   servo_control(angles[i], i);
    }
          
    delay(10);
}
