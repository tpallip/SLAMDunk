/**
 * @brief The simulated robot body, motors, encoders and surface sensors.
 * @details truth is where the robot really is. The controller never sees it;
 * it gets encoder counts and sensor readings, like on the real robot.
 */
class Robot_c {
  final float bodyRadiusMm = 50.0;
  final float COUNTS_PER_REV = 358.3;
  final float[] SENSOR_ANGLES = {45, 14, 0, -14, -45};

  float leftRadiusMm, rightRadiusMm, separationMm;
  long simulationMillis = 0;
  PImage surfaceImage;
  final float surfaceOffsetX;
  final float surfaceOffsetY;
  RobotSurface_c surface = new RobotSurface_c();
  MotorsModel_c motors;
  SurfaceSensorModel_c[] sensors = new SurfaceSensorModel_c[5];

  RobotPose_c truth = new RobotPose_c();
  float leftCounts = 0, rightCounts = 0;  // the encoders report the whole part

  Robot_c(JSONObject cfg, PImage surfaceIn, float surfaceOffsetXIn, float surfaceOffsetYIn) {
    leftRadiusMm = cfg.getFloat("left_wheel_radius_mm", 16.0);
    rightRadiusMm = cfg.getFloat("right_wheel_radius_mm", 16.0);
    separationMm = cfg.getFloat("wheel_separation_mm", 90.0);
    surfaceImage = surfaceIn;
    surfaceOffsetX = surfaceOffsetXIn;
    surfaceOffsetY = surfaceOffsetYIn;
    motors = new MotorsModel_c(cfg);
    // measured sensor levels, the same ones the robot normalises with
    for (int i = 0; i < 5; i++) {
      String dn = "dn" + (i + 1);
      sensors[i] = new SurfaceSensorModel_c(dn.toUpperCase(), 45, SENSOR_ANGLES[i],
        cfg.getFloat(dn + "_white", 400), cfg.getFloat(dn + "_black", 2000));
    }
    resetForRun(0, 0, -PI / 2);
  }

  long getMillis() { return simulationMillis; }
  void advanceTime(int dtMs) { simulationMillis += dtMs; }

  void getSurfaceSensors() {
    for (int i = 0; i < sensors.length; i++) {
      surface.reading[i] = sensors[i].sample(surfaceImage, surfaceOffsetX, surfaceOffsetY,
        truth.x, truth.y, truth.theta);
    }
  }

  int getLeftEncoderCount() { return floor(leftCounts); }
  int getRightEncoderCount() { return floor(rightCounts); }

  void setMotorPWM(float leftPwm, float rightPwm) {
    motors.setPWM(leftPwm, rightPwm);
  }

  // Put the robot down at a pose, stopped. The encoders keep their counts.
  void resetForRun(float x, float y, float theta) {
    truth.set(x, y, theta);
    motors.reset();
  }

  void updatePhysics(int dtMs) {
    motors.update(dtMs);
    float left = motors.left.speedMmS * dtMs / 1000.0;
    float right = motors.right.speedMmS * dtMs / 1000.0;
    leftCounts += left / (TWO_PI * leftRadiusMm) * COUNTS_PER_REV;
    rightCounts += right / (TWO_PI * rightRadiusMm) * COUNTS_PER_REV;

    float distance = (left + right) / 2.0;
    truth.x += distance * cos(truth.theta);
    truth.y += distance * sin(truth.theta);
    truth.theta += (right - left) / separationMm;
  }

  float[] sensorReadings() {
    float[] r = new float[5];
    for (int i = 0; i < 5; i++) r[i] = surface.reading[i];
    return r;
  }

  void draw() {
    pushMatrix();
    translate(truth.x, truth.y);
    rotate(truth.theta);
    fill(190, 205, 220);
    stroke(40);
    strokeWeight(2);
    ellipse(0, 0, bodyRadiusMm, bodyRadiusMm);
    stroke(40, 100, 220);
    strokeWeight(3);
    line(0, 0, bodyRadiusMm * .72, 0);
    fill(40, 100, 220);
    noStroke();
    triangle(bodyRadiusMm * .92, 0, bodyRadiusMm * .62, bodyRadiusMm * .14,
      bodyRadiusMm * .62, -bodyRadiusMm * .14);
    for (int i = 0; i < sensors.length; i++) sensors[i].draw(surface.reading[i]);
    popMatrix();
  }
}
