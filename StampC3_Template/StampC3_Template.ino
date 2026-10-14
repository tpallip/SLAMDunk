#include <Wire.h>

#include "RobotWifiAP.h"

/** @brief This robot's name and WiFi network. Set once with "name <new name>";
 * until then it's "SLAMDunk-" plus the end of the chip's MAC address. */
char robot_id[24];

/** @brief WiFi access-point and TCP server used for optional telemetry. */
RobotWifiAP_c server(robot_id, "lokum123", 80);

#include "Robot.h"
#include "RobotIMU.h"
#include "Controller.h"


/** @brief Hardware-facing interface to the Pololu robot. */
Robot_c robot;
/** @brief Direct interface to the 3Pi+ accelerometer, gyro, and magnetometer. */
RobotIMU_c imu;
/** @brief Supplied nominal line-following controller. */
Controller_c controller;


/** @brief Initialises serial, WiFi, IMU, and the supplied robot-controller baseline. */
void setup() {

  Serial.begin(115200);
  if (!RobotConfig_c::loadName(robot_id, sizeof(robot_id))) {
    uint64_t mac = ESP.getEfuseMac();
    snprintf(robot_id, sizeof(robot_id), "SLAMDunk-%02X%02X",
             (unsigned)((mac >> 32) & 0xFF), (unsigned)((mac >> 40) & 0xFF));
  }
  server.begin();

  robot.initialise("SLAMDunk");
//  robot.initialise("team name");

  controller.begin(robot_id);


  // Robot_c starts the shared I2C bus before RobotIMU_c uses it. IMU readings
  // come directly from the sensor ICs and do not pass through the middleware.
  if (!imu.initialise() ) {
    Serial.println("Warning: LSM6DS33 IMU was not detected.");
    robot.setLED( 90, 50, 0, 100 ); // orange warning
  }

}


/** @brief Maintains telemetry transport and runs the controller repeatedly. */
void loop() {
  // Accept or maintain the optional Processing telemetry connection. The
  // controller continues to run even when no client is connected.
  server.update();

  
  // Run an iteration of the robot controller code.
  // Develop controller routines in Controller.cpp; edit parameters in Controller.h.
  controller.update(robot, server);

}
