#include <Arduino.h>
#include <decode_packet.h>
#include <encoder.h>
#include <Kinematic.h>
#include <my_remotexy.h>
#include <ota.h>
#define LED 2

void setup() {
    Serial.begin(115200);
    Serial.println("Start Program");
    setup_servo();
    encoder_setup();
    setup_ota();
    // remotexy_setup();
    for(int i=0; i < NUM_MOTORS; i++)  servo_control(45, i); 
    RobotArm arm = setup_kin();
    use_example(arm);
    pinMode(LED,OUTPUT);
}


void loop() {

    serial_listener();
    loop_ota();

    // remotexy_loop();
    // if (RemoteXY.connect_flag)
    // {   
    //   servo_control(remotexy2angle(RemoteXY.Base), 0);
    //   servo_control(remotexy2angle(RemoteXY.Shoulder), 1);
    //   servo_control(remotexy2angle(RemoteXY.wrist1), 2);
    //   servo_control(remotexy2angle(RemoteXY.wrist2), 3);
    //   servo_control(remotexy2angle(RemoteXY.wrist3), 4);
    //   servo_control(remotexy2angle(RemoteXY.Base), 5);
    //   if (RemoteXY.Gripper)
    //     servo_control(0, 6);
    //   else
    //     servo_control(SERVO_MAX_DEGREE, 6);
    // }
    // else  // Encoders control the motors
    // {
      std::array<int, NUM_MOTORS> angles = encoder_loop();
      for(int i=0; i < NUM_MOTORS; i++)   servo_control(angles[i], i);   // 
    // }
    digitalWrite(LED,HIGH);
    delay(50);
    digitalWrite(LED,LOW);
    delay(50);
}
