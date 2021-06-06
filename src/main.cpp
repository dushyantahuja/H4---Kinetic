#include "Arduino.h"
#define H4P_VERBOSE 1
#include <H4.h>
#include <H4Plugins.h>
#include <AccelStepper.h>

#define EN_PIN           D1 // Enable
#define DIR_PIN          D3 // Direction
#define STEP_PIN         D4 // Step
constexpr uint32_t steps_per_mm = 80;
uint maxSpeed = 5;
uint accel = 2;

AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);
int direction = 1;
bool running = false;

void onWiFiConnect( void );
void onWiFiDisconnect( void );

H4_USE_PLUGINS(115200,H4_Q_CAPACITY,true) // Serial baud rate, Q size, SerialCmd autostop

H4P_WiFi h4wifi("DUSHYANT", "ahuja987", "Kinetic");   // Configure and activate the WiFi Interface.
H4P_BinaryThing h4onof([](bool b){
  if(b){
    running = true;
    stepper.move(100*steps_per_mm*direction); // Move 100mm
    //if(random(10)>=5) direction = -direction;
    stepper.enableOutputs();
  }
  else{
    running = false;
    stepper.stop();
    stepper.disableOutputs();
  }
  },OFF,0);
H4P_UPNPServer lexy("Kinetic");

//******************************************************************************
//*** Main setup routine.
//******************************************************************************

void h4setup( void ) {
  randomSeed(analogRead(A0));
  stepper.setMaxSpeed(maxSpeed*steps_per_mm); // 20mm/s @ 80 steps/mm
  stepper.setAcceleration(accel*steps_per_mm); // 5mm/s^2
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);
  //stepper.enableOutputs();

  /*h4.every(1000,[](){ 
    Serial.println(stepper.distanceToGo());
    if (stepper.distanceToGo() == 0) {
        stepper.disableOutputs();
        if(running) {
            stepper.move(random(100)*steps_per_mm*direction); // Move 100mm
            if(random(10)>=5) direction = -direction;
            stepper.enableOutputs();
        }
    }
  });*/
}

void onReboot(){
    Serial.println("This always gets called LAST and without you needing to 'hook' it");
}

void h4UserLoop(){
  stepper.run();
}