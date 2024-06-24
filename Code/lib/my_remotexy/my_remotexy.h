#ifndef _myremotexy_h
#define _myremotexy_h
// you can enable debug logging to Serial at 115200
// #define REMOTEXY__DEBUGLOG    
// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__ESP32CORE_BLE
#include <BLEDevice.h>
// RemoteXY connection settings 
#define REMOTEXY_BLUETOOTH_NAME "Manipulator"
#include <RemoteXY.h>

// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 85 bytes
  { 255,7,0,0,0,78,0,17,0,0,0,24,2,106,200,200,84,1,1,4,
  0,5,10,87,41,41,17,19,35,35,0,2,26,31,5,11,37,39,39,64,
  19,35,35,0,2,26,31,5,61,34,39,39,110,19,35,35,0,2,26,31,
  10,62,141,57,57,155,20,33,33,48,65,26,31,67,108,111,115,101,0,31,
  79,112,101,110,0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  int8_t Base; // =-100..100 slider position
  int8_t Shoulder; // =-100..100 slider position
  int8_t Elbow; // =-100..100 slider position
  int8_t wrist1; // =-100..100 slider position
  int8_t wrist2; // =-100..100 slider position
  int8_t wrist3; // =-100..100 slider position
  int8_t Gripper; // =1 if button pressed, else =0

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   
#pragma pack(pop)
 
 void remotexy_setup()
 {
      RemoteXY_Init(); 
 }

 void remotexy_loop()
 {
  RemoteXY_Handler();
 }
 #endif