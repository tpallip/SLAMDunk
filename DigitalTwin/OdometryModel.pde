class RobotPose_c {
  float x = 0;
  float y = 0;
  float theta = 0;

  void set(float xIn, float yIn, float thetaIn) {
    x = xIn;
    y = yIn;
    theta = thetaIn;
  }
}

/**
 * @brief Pose from encoder counts, same maths as the StampC3's OdometryModel.
 * @note It only sees the counts, so it can drift from the true pose.
 */
class OdometryModel_c {
  final float COUNTS_PER_REV = 358.3;
  float leftRadiusMm, rightRadiusMm, separationMm;
  RobotPose_c pose = new RobotPose_c();
  boolean started = false;
  int lastLeft = 0, lastRight = 0;

  OdometryModel_c(float leftRadiusMmIn, float rightRadiusMmIn, float separationMmIn) {
    leftRadiusMm = leftRadiusMmIn;
    rightRadiusMm = rightRadiusMmIn;
    separationMm = separationMmIn;
  }

  void reset(float x, float y, float theta) {
    pose.set(x, y, theta);
    started = false;
  }

  void update(int leftCount, int rightCount) {
    if (!started) {
      lastLeft = leftCount;
      lastRight = rightCount;
      started = true;
      return;
    }
    float left = (leftCount - lastLeft) * TWO_PI * leftRadiusMm / COUNTS_PER_REV;
    float right = (rightCount - lastRight) * TWO_PI * rightRadiusMm / COUNTS_PER_REV;
    lastLeft = leftCount;
    lastRight = rightCount;

    float distance = (left + right) / 2.0;
    pose.x += distance * cos(pose.theta);
    pose.y += distance * sin(pose.theta);
    pose.theta += (right - left) / separationMm;
  }
}
