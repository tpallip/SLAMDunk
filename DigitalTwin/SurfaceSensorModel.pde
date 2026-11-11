class RobotSurface_c {
  int[] reading = new int[5];
}

/**
 * @brief One surface sensor: reads the map under it and scales from its own
 * white level to its own black level (dnN_white/dnN_black under "_surface" in lokum.json),
 * plus a little noise.
 */
class SurfaceSensorModel_c {
  // about 14-22 counts on the robot (bench/3.1_static_noise)
  final float NOISE = 15;

  String name;
  float localX;
  float localY;
  float white, black;

  SurfaceSensorModel_c(String nameIn, float radiusMm, float angleDegrees, float whiteIn, float blackIn) {
    name = nameIn;
    localX = radiusMm * cos(radians(angleDegrees));
    localY = radiusMm * sin(radians(angleDegrees));
    white = whiteIn;
    black = blackIn;
  }

  int sample(PImage image, float surfaceOffsetX, float surfaceOffsetY,
    float robotX, float robotY, float robotTheta) {
    float worldX = robotX + localX * cos(robotTheta) - localY * sin(robotTheta);
    float worldY = robotY + localX * sin(robotTheta) + localY * cos(robotTheta);
    int pixelX = floor(image.width / 2.0 + worldX - surfaceOffsetX);
    int pixelY = floor(image.height / 2.0 - worldY + surfaceOffsetY);

    float darkness = 0;  // off the map counts as white
    if (pixelX >= 0 && pixelX < image.width && pixelY >= 0 && pixelY < image.height) {
      darkness = 1 - brightness(image.get(pixelX, pixelY)) / 255.0;
    }
    return round(white + darkness * (black - white) + randomGaussian() * NOISE);
  }

  void draw(float reading) {
    pushStyle();
    fill(map(reading, white, black, 220, 20));
    stroke(20);
    rectMode(CENTER);
    rect(localX, localY, 3, 3);
    popStyle();
  }
}
