/**
 * BICOPTER with altitude control
 * This code runs a bicopter with altitude control using the feedback from a barometer.
 * For this example, your robot needs a barometer sensor.
 */

#include "BlimpSwarm.h"
#include "robot/RobotFactory.h"
#include "state/nicla/NiclaConfig.h"
#include "comm/BaseCommunicator.h"
#include "comm/LLC_ESPNow.h"
#include "util/Print.h"
#include "sense/SensorSuite.h"
#include <Arduino.h>
#include <ESP32Servo.h>


// Robot
Robot* myRobot = nullptr;


nicla_t terms; 

// Communication
BaseCommunicator* baseComm = nullptr;

// Control input from base station
ControlInput behave;
ControlInput cmd;
ReceivedData rcv;

// Data storage for the sensors
float senses[myRobot->MAX_SENSORS];

bool updateParams = true;
const int TIME_STEP_MICRO = 4000;

int dt = 1000;
unsigned long clockTime;
unsigned long printTime;

int niclaOffset = 11;


void setup() {
    Serial.begin(115200);
    Serial.println("Start!");
    clockTime = micros();
    printTime = micros();
    // init communication
    baseComm = new BaseCommunicator(new LLC_ESPNow());
    baseComm->setMainBaseStation();

    // init robot with new parameters
    myRobot = RobotFactory::createRobot("FullBicopter");
    
    paramUpdate();

    // updates the ground altitude for the ground feedback
    // TODO: make some way to access the actual ground height from robot
    int numSenses = myRobot->sense(senses);



}

float nicla_yaw = 0;
float z_estimator = 0;
float forward_force = 0;
unsigned long SpinTimer = millis();
float SpinYaw = 0.0;
const float SPIRAL_INCREMENT = 0.08; 

void loop() {
  // Retrieves cmd.params from ground station and checks flags
  recieveCommands();

  // Get sensor values
  int numSenses = myRobot->sense(senses);

  // send values to ground station
  rcv.flag = 1;
  rcv.values[0] = senses[1];  //height
  rcv.values[1] = senses[5];  //yaw
  rcv.values[2] = senses[niclaOffset + 1];  //nicla x
  rcv.values[3] = senses[niclaOffset + 2];  //nicla y
  rcv.values[4] = senses[niclaOffset + 3];  //nicla w
  rcv.values[5] = senses[niclaOffset + 4];  //nicla h
  bool sent = baseComm->sendMeasurements(&rcv);

  // print sensor values every second
  // senses => [temperature, altitude, veloctity_in_altitude, roll, pitch, yaw, rollrate, pitchrate, yawrate, null, battery, nicla_flag, nicla_x, nicla_y, nicla_w, nicla_h, nicla...]
  if (micros() - printTime > 515106){
    
    for (int i = 0; i < numSenses-1; i++){
      Serial.print(senses[i]);
      Serial.print(",");
    }
    Serial.println(senses[numSenses-1]);
    printTime = micros();
  }

  const float YAW_INCREMENT = 0.2; // Adjust this value based on desired turning speed
  unsigned long lastBalloonDetectionTime = 0;
  const unsigned long TURNING_DELAY = 3000; // 3000 milliseconds or 3 seconds
  bool shouldStartTurning = false;

  // Nicla controller (when the incomming flag = 2)
  if (cmd.params[0] == 2) {
    int nicla_flag = (int)senses[niclaOffset + 0];
    if (nicla_flag != 0) {// checks if new yaw occurs
        // lastBalloonDetectionTime = millis(); // Update the last detection time
        // shouldStartTurning = false; // Reset turning flag
        float _yaw = senses[5];  
        float _height = senses[1];  
        float tracking_x = (float)senses[niclaOffset + 1];
        float tracking_y = (float)senses[niclaOffset + 2];
        
        float x_cal = tracking_x / terms.n_max_x; // normalizes the pixles into a value between [0,1]
        float des_yaw = ((x_cal - 0.5)) * terms.x_strength; // normalizes the normal to between [-.5, .5] to act as an offset for yaw
        nicla_yaw = _yaw + des_yaw; // add the offset in yaw to the current yaw for movement.
        float y_cal = tracking_y / terms.n_max_y;
        if ( abs(x_cal - 0.5) < .16 && terms.y_strength != 0) { // makes sure yaw is in center before making height adjustments
            z_estimator =  ( _height + terms.y_strength * (y_cal - terms.y_thresh)) ; // height control doenst work well when not 0 bouyant
            forward_force = 0.16;
        } else {
          forward_force = 0;
        }
    } else{ // nical flag does not see red
      // forward_force = 0.13;
      // nicla_yaw += YAW_INCREMENT; // Incrementally turn the bicopter
      // if (nicla_yaw > 360) nicla_yaw -= 360; // Keep the yaw value within 0-360 degrees
      // if (millis() - lastBalloonDetectionTime > TURNING_DELAY) {
      //       shouldStartTurning = true;
      // }

      //   if (shouldStartTurning) {
      //       forward_force = 0.13;
      //       nicla_yaw += YAW_INCREMENT; // Incrementally turn the bicopter
      //       if (nicla_yaw > 360) nicla_yaw -= 360; // Normalize yaw
      //   } else {
      //       forward_force = 0.1; // Possibly halt movement until turning starts
      //   }
      // behave.params[2] = cmd.params[1]; // fz (meters)
      int dt = SpinTimer - millis();
      SpinTimer += dt;
      SpinYaw += (float)(1000 / dt) * SPIRAL_INCREMENT;
      // if (SpinYaw > 1) SpinYaw -= 1;
      forward_force = 0.13;
    }

    behave.params[0] = cmd.params[0]; // flag
    behave.params[1] = forward_force; // fx ('meters'/second)
    behave.params[2] = z_estimator; // fz (meters)
    behave.params[3] = 0; // tx (radians/second)
    behave.params[4] = nicla_yaw + SpinYaw; // tz (radians)

  } else { // direct control with joystick if 'flag' is not 2
    z_estimator = cmd.params[2];
    nicla_yaw = cmd.params[4]; // autoset for when switch occurs
    behave.params[0] = cmd.params[0]; //flag
    behave.params[1] = cmd.params[1]; //fx
    behave.params[2] = cmd.params[2]; //fz
    behave.params[3] = cmd.params[3]; //tx
    behave.params[4] = cmd.params[4]; //tz
  }

  // Send command to the actuators
  myRobot->control(senses, behave.params, 5);

  // makes the clock rate of the loop consistant.
  fixClockRate();
}

void recieveCommands(){
  if (baseComm->isNewMsgCmd()){
    // New command received
    cmd = baseComm->receiveMsgCmd();
    if (int(cmd.params[11]) == 1 && updateParams){
      paramUpdate();
      updateParams = false;
    } else {
      updateParams = true;
    }
    // Print command
    Serial.print("Cmd arrived: ");
    printControlInput(cmd);
  }
}

void paramUpdate(){
    NiclaConfig::getInstance()->loadConfiguration();
    const nicla_t& config = NiclaConfig::getInstance()->getConfiguration();
    terms = config; // Copy configuration data
    myRobot->getPreferences();
    baseComm->setMainBaseStation();

}

void fixClockRate() {

  dt = (int)(micros()-clockTime);
  while (TIME_STEP_MICRO - dt > 0){
    dt = (int)(micros()-clockTime);
  }
  clockTime = micros();
}