/**
 * @brief Owns the standalone application: timing, robot model, view, and TCP.
 * @details World positions use millimetres, with +x forward/right and +y
 * upward. Rendering may run at a variable frame rate; model/controller
 * updates use a fixed 10 ms interval.
 */
class Simulator {
  /** @brief Number of milliseconds visible in each telemetry graph. */
  final int graphWindowMs = 5000;
  final int graphSamplePeriodMs = 20;
  // Mirrors the physical robot's 10 ms update interval.
  final int simulationUpdateIntervalMs = 10;
  final int maximumCatchUpSteps = 10;
  /**
   * @brief Where live-comparison telemetry comes from.
   * @details true: the logger's hub (tools/logger.py, 127.0.0.1:9200), so the
   * logger and the twin run together. false: straight to the StampC3's access
   * point (192.168.4.1:80), which takes one client, so not with the logger.
   */
  final boolean useLoggerHub = true;
  final String tcpHost = useLoggerHub ? "127.0.0.1" : "192.168.4.1";
  final int tcpPort = useLoggerHub ? 9200 : 80;
  final int tcpRetryPeriodMs = 2000;
  /**
   * @brief Map pose where the real robot is placed before a live run: on the
   * left line, halfway down, facing down the page. Same as Robot_c's start.
   * @details The robot's odometry starts at (0, 0, 0) wherever it is put down,
   * so its x axis points the way it faces. Live poses are taken relative to
   * the robot's pose at start and placed on the map from here; see toMap().
   */
  final float homeX = 0.0;
  final float homeY = 0.0;
  float homeTheta = -PI / 2.0;
  final int tcpConnectTimeoutMs = 250;
  final float minimumZoom = 0.25;
  final float maximumZoom = 4.0;
  final int SANDBOX = 0, LIVE_COMPARISON = 1;
  final String robotId = "lokum";
  final String logDir = "../logs";

  /** @brief Surface-image centre in world coordinates, in mm. */
  final float surfaceOffsetX;
  final float surfaceOffsetY;

  PImage surfaceImage;
  // Cache only the currently visible part of the 1 px/mm map. This avoids
  // redrawing the complete source image at 1:1 scale.
  PImage surfaceViewportCache;
  int surfaceCacheX = -1;
  int surfaceCacheY = -1;
  int surfaceCacheWidth = 0;
  int surfaceCacheHeight = 0;
  PApplet applet;
  /** @brief Robot-only Digital Twin model; the surface image is its environment. */
  Robot_c robot;
  Controller_c controller;
  TimeSeriesGraph motorGraph;
  TimeSeriesGraph sensorGraph;
  TimeSeriesGraph encoderGraph;

  boolean showTelemetry = false;
  boolean trackRobot = false;
  int simulatorMode = SANDBOX;
  boolean liveArmed = false;
  boolean liveRunning = false;
  int previousTcpSignal = 0;
  long liveStartTcpMs = -1;
  long liveStartSimulationMs = -1;
  long lastTcpTimestampMs = -1;
  TCPTelemetry_c armedPose = new TCPTelemetry_c();
  TCPTelemetry_c ghost = new TCPTelemetry_c();
  ArrayList<TCPTelemetry_c> tcpQueue = new ArrayList<TCPTelemetry_c>();
  float zoom = 1.0;
  float viewCentreX = 0.0;
  float viewCentreY = 0.0;

  int lastGraphSampleMs = 0;
  int lastSimulationUpdateMs = 0;
  int simulationAccumulatorMs = 0;
  int lastTcpAttemptMs = -tcpRetryPeriodMs;

  PrintWriter runLog = null;
  String runLogName = "";

  Client telemetryClient;
  TCPTelemetry_c tcpTelemetry = new TCPTelemetry_c();
  String tcpStatus = "TCP: not connected";
  String lastTcpLine = "";
  volatile boolean tcpAttemptInProgress = false;
  volatile boolean tcpAttemptFailed = false;
  volatile Socket pendingTcpSocket = null;

  /**
   * @brief Creates the supplied baseline Digital Twin application.
   * @param appletIn Parent Processing application used for networking.
   * @param surfaceOffsetXIn Surface-image x offset in world millimetres.
   * @param surfaceOffsetYIn Surface-image y offset in world millimetres.
   */
  Simulator(PApplet appletIn, float surfaceOffsetXIn, float surfaceOffsetYIn) {
    applet = appletIn;
    surfaceOffsetX = surfaceOffsetXIn;
    surfaceOffsetY = surfaceOffsetYIn;
    ellipseMode(RADIUS);
    strokeJoin(ROUND);
    surfaceImage = loadSurfaceImage();
    // same settings file the logger sends to the robot
    JSONObject cfg = applet.loadJSONObject(applet.sketchPath("../config/" + robotId + ".json"));
    robot = new Robot_c(cfg, surfaceImage, surfaceOffsetX, surfaceOffsetY);
    controller = new Controller_c(cfg);
    startRun(); // Sandbox starts immediately.
    lastSimulationUpdateMs = millis();

    motorGraph = new TimeSeriesGraph("Motor PWM", graphWindowMs, true);
    motorGraph.addSeries("Sim L", color(220, 70, 70));
    motorGraph.addSeries("Sim R", color(70, 100, 220));
    motorGraph.addSeries("TCP L", color(145, 60, 190));
    motorGraph.addSeries("TCP R", color(20, 150, 150));
    sensorGraph = new TimeSeriesGraph("Surface sensors", graphWindowMs, false);
    for (int i = 0; i < 5; i++) sensorGraph.addSeries("Sim " + (i + 1), sensorColour(i));
    for (int i = 0; i < 5; i++) sensorGraph.addSeries("TCP " + (i + 1), tcpSensorColour(i));
    encoderGraph = new TimeSeriesGraph("Encoder counts", graphWindowMs, false);
    encoderGraph.addSeries("Sim L", color(220, 70, 70));
    encoderGraph.addSeries("Sim R", color(70, 100, 220));
    encoderGraph.addSeries("TCP L", color(145, 60, 190));
    encoderGraph.addSeries("TCP R", color(20, 150, 150));

  }

  /** @brief Updates TCP, advances fixed-step model time, and records graphs. */
  void update() {
    updateTcpConnection();
    updateRobotModel();

    if (trackRobot) {
      viewCentreX = robot.truth.x;
      viewCentreY = robot.truth.y;
    }
    recordTelemetry();
  }

  /**
   * @brief Accumulates elapsed wall-clock time into fixed model updates.
   * @note Long pauses are capped to prevent an unbounded catch-up simulation.
   */
  void updateRobotModel() {
    int now = millis();
    int elapsedMs = now - lastSimulationUpdateMs;
    lastSimulationUpdateMs = now;

    // Do not try to replay an unbounded amount of time after a breakpoint or
    // long application pause.
    elapsedMs = min(elapsedMs, simulationUpdateIntervalMs * maximumCatchUpSteps);
    simulationAccumulatorMs += elapsedMs;

    int steps = 0;
    while (simulationAccumulatorMs >= simulationUpdateIntervalMs
      && steps < maximumCatchUpSteps) {
      step();
      simulationAccumulatorMs -= simulationUpdateIntervalMs;
      steps++;
    }
  }

  // one 10 ms model step
  void step() {
    robot.advanceTime(simulationUpdateIntervalMs);
    updateLiveClock();
    controller.update(robot);
    logTelemetry();
    robot.updatePhysics(simulationUpdateIntervalMs);
  }

  // Runs from the home pose as fast as the computer allows, one log each.
  // Run i seeds the sensor noise with i, so any run can be repeated exactly.
  void runBatch(int runs, int seconds) {
    String stamp = timestamp();
    for (int i = 0; i < runs; i++) {
      randomSeed(i);
      String way = homeTheta > 0 ? "cw" : "acw";
      startRun("twin_" + stamp + "_" + way + "_seed" + i + ".csv");
      for (int k = 0; k < seconds * 1000 / simulationUpdateIntervalMs; k++) step();
      closeRunLog();
      println("wrote " + runLogName);
    }
  }

  /** @brief Advances the physical-robot ghost only to matching telemetry time. */
  void updateLiveClock() {
    if (simulatorMode != LIVE_COMPARISON || !liveRunning) return;
    long targetTcpTime = liveStartTcpMs + (robot.getMillis() - liveStartSimulationMs);
    while (tcpQueue.size() > 0 && tcpQueue.get(0).timestampMs <= targetTcpTime) {
      ghost = tcpQueue.remove(0);
    }
  }

  /** @brief Renders the surface, simulated robot, optional ghost, and UI. */
  void draw() {

    background(180);
    updateSurfaceViewportCache();

    pushMatrix();
    translate(width / 2.0, height / 2.0);
    scale(zoom, -zoom);
    translate(-viewCentreX, -viewCentreY);
    imageMode(CENTER);
    drawSurfaceViewportCache();
    robot.draw();
    if (simulatorMode == LIVE_COMPARISON && ghost.timestampMs >= 0) drawGhost();

    popMatrix();

    drawStatus();
    if (showTelemetry) drawTelemetryPanel();
  }

  void updateSurfaceViewportCache() {
    // Convert the visible world rectangle to source-image pixels. The image
    // is centred on the configured offset, and its y axis is opposite the
    // world y axis.
    float halfViewWidth = width / (2.0 * zoom);
    float halfViewHeight = height / (2.0 * zoom);
    int visibleX0 = floor(surfaceImage.width / 2.0
      + viewCentreX - surfaceOffsetX - halfViewWidth);
    int visibleX1 = ceil(surfaceImage.width / 2.0
      + viewCentreX - surfaceOffsetX + halfViewWidth);
    int visibleY0 = floor(surfaceImage.height / 2.0
      - viewCentreY + surfaceOffsetY - halfViewHeight);
    int visibleY1 = ceil(surfaceImage.height / 2.0
      - viewCentreY + surfaceOffsetY + halfViewHeight);

    visibleX0 = constrain(visibleX0, 0, surfaceImage.width);
    visibleX1 = constrain(visibleX1, 0, surfaceImage.width);
    visibleY0 = constrain(visibleY0, 0, surfaceImage.height);
    visibleY1 = constrain(visibleY1, 0, surfaceImage.height);
    if (visibleX1 <= visibleX0 || visibleY1 <= visibleY0) {
      surfaceViewportCache = null;
      return;
    }

    boolean cacheContainsView = surfaceViewportCache != null
      && visibleX0 >= surfaceCacheX
      && visibleY0 >= surfaceCacheY
      && visibleX1 <= surfaceCacheX + surfaceCacheWidth
      && visibleY1 <= surfaceCacheY + surfaceCacheHeight;
    if (cacheContainsView) return;

    // The margin lets the robot move or the view pan a little before a new
    // crop is made. It keeps cache creation out of the normal draw path.
    int cacheMargin = ceil(constrain(max(halfViewWidth, halfViewHeight) * 0.15,
      40, 250));
    surfaceCacheX = max(0, visibleX0 - cacheMargin);
    surfaceCacheY = max(0, visibleY0 - cacheMargin);
    int cacheRight = min(surfaceImage.width, visibleX1 + cacheMargin);
    int cacheBottom = min(surfaceImage.height, visibleY1 + cacheMargin);
    surfaceCacheWidth = cacheRight - surfaceCacheX;
    surfaceCacheHeight = cacheBottom - surfaceCacheY;
    surfaceViewportCache = surfaceImage.get(surfaceCacheX, surfaceCacheY,
      surfaceCacheWidth, surfaceCacheHeight);
  }

  void drawSurfaceViewportCache() {
    if (surfaceViewportCache == null) return;

    float worldX = surfaceOffsetX + surfaceCacheX + surfaceCacheWidth / 2.0
      - surfaceImage.width / 2.0;
    float worldY = surfaceOffsetY + surfaceImage.height / 2.0
      - (surfaceCacheY + surfaceCacheHeight / 2.0);
    image(surfaceViewportCache, worldX, worldY);
  }

  void drawGhost() {
    float alpha = millis() - ghost.receivedAtMs > 100 ? 128 : 255;
    pushStyle(); pushMatrix();
    translate(ghost.x, ghost.y); rotate(ghost.theta);
    fill(235, 125, 35, alpha); stroke(180, 70, 15, alpha);
    ellipse(0, 0, 50, 50); line(0, 0, 36, 0);
    popMatrix(); popStyle();
  }

  /**
   * @brief Handles the documented keyboard controls.
   * @param pressedKey Character key reported by Processing.
   * @param pressedKeyCode Processing key code for arrow-key panning.
   */
  void handleKey(char pressedKey, int pressedKeyCode) {
    if (pressedKey == 'g' || pressedKey == 'G') showTelemetry = !showTelemetry;
    if (pressedKey == 't' || pressedKey == 'T') trackRobot = !trackRobot;
    if ((pressedKey == 'm' || pressedKey == 'M') && !liveRunning) toggleMode();
    if ((pressedKey == 'r' || pressedKey == 'R') && simulatorMode == SANDBOX) startRun();
    if (pressedKey == '+' || pressedKey == '=') zoom = min(maximumZoom, zoom * 1.2);
    if (pressedKey == '-') zoom = max(minimumZoom, zoom / 1.2);

    if (pressedKey == CODED) {
      // Arrow-key panning intentionally disables tracking so the requested
      // view remains visible after the key press.
      float panStepMm = 50.0 / zoom;
      if (pressedKeyCode == LEFT) panBy(-panStepMm, 0);
      if (pressedKeyCode == RIGHT) panBy(panStepMm, 0);
      if (pressedKeyCode == UP) panBy(0, panStepMm);
      if (pressedKeyCode == DOWN) panBy(0, -panStepMm);
    }
  }

  /** @brief Switches between standalone sandbox and live-comparison modes. */
  void toggleMode() {
    simulatorMode = simulatorMode == SANDBOX ? LIVE_COMPARISON : SANDBOX;
    resetLiveSession();
    closeRunLog();
    if (simulatorMode == SANDBOX) startRun();
  }

  // like pressing the button with the robot on the home spot
  void startRun() {
    startRun("twin_" + timestamp() + ".csv");
  }

  void startRun(String logName) {
    robot.resetForRun(homeX, homeY, homeTheta);
    controller.reset();
    controller.odometry.reset(homeX, homeY, homeTheta);
    controller.setSignal(1);
    openRunLog(logName);
  }

  String timestamp() {
    return nf(year(), 4) + nf(month(), 2) + nf(day(), 2) + "_"
      + nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2);
  }

  void openRunLog(String logName) {
    closeRunLog();
    runLogName = logName;
    runLog = createWriter(applet.sketchPath(logDir + "/" + runLogName));
    runLog.println("t_ms,x_mm,y_mm,theta_rad,pwm_left,pwm_right,enc_left,enc_right,"
      + "dn1,dn2,dn3,dn4,dn5,signal,true_x_mm,true_y_mm,true_theta_rad");
  }

  void closeRunLog() {
    if (runLog == null) return;
    runLog.flush();
    runLog.close();
    runLog = null;
  }

  void logTelemetry() {
    if (runLog == null || controller.telemetryLine == null) return;
    runLog.println(controller.telemetryLine + ","
      + String.format(java.util.Locale.ROOT, "%.2f,%.2f,%.5f", robot.truth.x, robot.truth.y, robot.truth.theta));
    runLog.flush();
  }

  void panBy(float deltaX, float deltaY) {
    trackRobot = false;
    viewCentreX += deltaX;
    viewCentreY += deltaY;
  }

  void drawStatus() {
    int margin = uiMargin();
    int contentWidth = statusWidth();
    float lineHeight = max(18, 16 * uiScale());

    pushStyle();
    rectMode(CORNER);
    noStroke();
    fill(255, 255, 255, 205);
    rect(margin - 6, margin - 6, contentWidth + 12, lineHeight * 4 + 12, 5);
    fill(30);
    textSize(max(12, 13 * uiScale()));
    text("True pose: " + nf(robot.truth.x, 0, 1) + ", " + nf(robot.truth.y, 0, 1) + ", "
      + nf(robot.truth.theta, 0, 2) + "   Odometry: " + nf(controller.odometry.pose.x, 0, 1) + ", "
      + nf(controller.odometry.pose.y, 0, 1) + ", " + nf(controller.odometry.pose.theta, 0, 2)
      + "   Log: " + (runLog == null ? "off" : runLogName),
      margin, margin + lineHeight, contentWidth, lineHeight);
    text("+/-: zoom   Arrows: pan   T: tracking "
      + (trackRobot ? "on" : "off") + "   G: telemetry   M: mode   R: restart run",
      margin, margin + lineHeight * 2, contentWidth, lineHeight);
    text(tcpStatus, margin, margin + lineHeight * 3, contentWidth, lineHeight);
    text(liveStatus(), margin, margin + lineHeight * 4, contentWidth, lineHeight);
    popStyle();
  }

  String liveStatus() {
    if (simulatorMode == SANDBOX) return "Mode: Sandbox";
    if (telemetryClient == null) return "Mode: Live comparison — Offline";
    if (!liveArmed) return "Mode: Live comparison — Awaiting fresh signal 0";
    if (!liveRunning) return "Mode: Live comparison — Armed";
    return "Mode: Live comparison — Running; ghost = robot-reported odometry";
  }

  /** @brief Adds current simulation and, when recent, TCP values to graphs. */
  void recordTelemetry() {
    int now = millis();
    if (now - lastGraphSampleMs < graphSamplePeriodMs) return;
    lastGraphSampleMs = now;
    float[] simulatedSensors = robot.sensorReadings();
    if (tcpTelemetry.isRecent(now)) {
    motorGraph.addSample(now, new float[] {robot.motors.left.pwm, robot.motors.right.pwm,
        tcpTelemetry.pwmLeft, tcpTelemetry.pwmRight});
      sensorGraph.addSample(now, combine(simulatedSensors, tcpTelemetry.sensorReadings));
      encoderGraph.addSample(now, new float[] {robot.getLeftEncoderCount(),
        robot.getRightEncoderCount(), tcpTelemetry.encoderLeft, tcpTelemetry.encoderRight});
    } else {
      motorGraph.addSample(now, new float[] {
        robot.motors.left.pwm, robot.motors.right.pwm
      });
      sensorGraph.addSample(now, simulatedSensors);
      encoderGraph.addSample(now, new float[] {
        robot.getLeftEncoderCount(), robot.getRightEncoderCount()
      });
    }
  }

  float[] combine(float[] first, float[] second) {
    float[] combined = new float[first.length + second.length];
    for (int i = 0; i < first.length; i++) combined[i] = first[i];
    for (int i = 0; i < second.length; i++) combined[first.length + i] = second[i];
    return combined;
  }

  void drawTelemetryPanel() {
    int margin = uiMargin();
    int panelWidth = telemetryPanelWidth();
    int panelX = width - panelWidth;
    int graphWidth = panelWidth - 2 * margin;
    float titleHeight = max(28, 24 * uiScale());
    float graphGap = margin;
    float graphHeight = (height - titleHeight - margin * 4 - graphGap * 2) / 3.0;

    // The panel deliberately fills the available height. This keeps graph
    // backgrounds attached to their plots after a resize, including when the
    // window is made smaller than its normal working size.
    graphHeight = max(1, graphHeight);
    float graphY = titleHeight + margin;
    pushStyle();
    rectMode(CORNER);
    noStroke();
    fill(248, 248, 248, 235);
    rect(panelX, 0, panelWidth, height);
    fill(30);
    textSize(max(12, 13 * uiScale()));
    text("Telemetry (press G to hide)", panelX + margin, titleHeight - margin / 2.0);
    motorGraph.draw(panelX + margin, graphY, graphWidth, graphHeight);
    graphY += graphHeight + graphGap;
    sensorGraph.draw(panelX + margin, graphY, graphWidth, graphHeight);
    graphY += graphHeight + graphGap;
    encoderGraph.draw(panelX + margin, graphY, graphWidth, graphHeight);
    popStyle();
  }

  float uiScale() {
    return constrain(min(width / 1000.0, height / 800.0), 0.85, 1.5);
  }

  int uiMargin() {
    return max(10, round(12 * uiScale()));
  }

  int telemetryPanelWidth() {
    int availableWidth = max(180, width - 2 * uiMargin());
    int preferredWidth = round(width * 0.32);
    return min(max(260, preferredWidth), availableWidth);
  }

  int statusWidth() {
    int availableRight = showTelemetry ? width - telemetryPanelWidth() : width;
    return max(1, availableRight - 2 * uiMargin());
  }

  color sensorColour(int index) {
    color[] colours = {color(160, 60, 60), color(210, 120, 50),
      color(80, 150, 80), color(60, 120, 190), color(140, 70, 170)};
    return colours[index];
  }

  color tcpSensorColour(int index) {
    color[] colours = {color(225, 110, 110), color(245, 180, 70),
      color(105, 190, 105), color(80, 160, 230), color(185, 100, 210)};
    return colours[index];
  }

  /**
   * @brief Loads surface.png or creates the supplied A3 circular-line fallback.
   * @return A 1 pixel/mm surface image positioned using the configured offset.
   */
  PImage loadSurfaceImage() {
    PImage image = loadImage("surface.png");
    if (image != null && image.width > 0 && image.height > 0) return image;

    // A3 landscape is 420 x 297 mm. At 1 px/mm this generated test surface
    // has the same physical dimensions as the printed student sheet.
    image = createImage(420, 297, RGB);
    image.loadPixels();
    float circleRadius = 125.0;
    float lineWidth = 19.0;
    for (int py = 0; py < image.height; py++) {
      for (int px = 0; px < image.width; px++) {
        float dx = px - image.width / 2.0;
        float dy = py - image.height / 2.0;
        float distanceFromCentre = sqrt(dx * dx + dy * dy);
        boolean onLine = abs(distanceFromCentre - circleRadius) <= lineWidth / 2.0;
        image.pixels[py * image.width + px] = onLine ? color(0) : color(255);
      }
    }
    image.updatePixels();
    return image;
  }

  /** @brief Maintains the TCP connection and consumes bounded CSV input. */
  void updateTcpConnection() {
    int now = millis();
    if (telemetryClient != null && !telemetryClient.active()) {
      telemetryClient.stop();
      telemetryClient = null;
      tcpStatus = "TCP: disconnected; retrying";
      resetLiveSession();
    }

    if (pendingTcpSocket != null) {
      Socket connectedSocket = pendingTcpSocket;
      pendingTcpSocket = null;
      tcpAttemptInProgress = false;
      try {
        telemetryClient = new Client(applet, connectedSocket);
        tcpStatus = "TCP: connected to " + tcpHost + ":" + tcpPort;
      }
      catch (Exception error) {
        telemetryClient = null;
        tcpStatus = "TCP: unavailable; retrying";
        closeSocket(connectedSocket);
      }
    }

    if (tcpAttemptFailed) {
      tcpAttemptFailed = false;
      tcpAttemptInProgress = false;
      tcpStatus = "TCP: unavailable; retrying";
    }

    if (telemetryClient == null && !tcpAttemptInProgress
      && now - lastTcpAttemptMs >= tcpRetryPeriodMs) {
      beginTcpConnectionAttempt();
    }

    int linesRead = 0;
    while (telemetryClient != null && telemetryClient.available() > 0 && linesRead < 20) {
      String line = telemetryClient.readStringUntil('\n');
      if (line == null) break;
      lastTcpLine = trim(line);
      if (tcpTelemetry.parse(lastTcpLine, now)) handleTcpSample(tcpTelemetry);
      linesRead++;
    }
  }

  /**
   * @brief Uses a parsed physical-robot record for graphing and live alignment.
   * @param sample Newly received StampC3 telemetry record.
   */
  void handleTcpSample(TCPTelemetry_c sample) {
    if (lastTcpTimestampMs >= 0 && sample.timestampMs < lastTcpTimestampMs) {
      resetLiveSession();
    }
    lastTcpTimestampMs = sample.timestampMs;
    TCPTelemetry_c copy = sample.copy();
    if (simulatorMode != LIVE_COMPARISON) return;

    if (liveRunning) tcpQueue.add(toMap(copy));

    if (sample.signal == 0) {
      armedPose = sample.copy();
      liveArmed = true;
    }
    if (liveArmed && !liveRunning && previousTcpSignal == 0 && sample.signal == 1) {
      startRun();
      liveStartTcpMs = sample.timestampMs;
      liveStartSimulationMs = robot.getMillis();
      ghost = toMap(armedPose);
      liveRunning = true;
      tcpQueue.clear();
    }
    previousTcpSignal = sample.signal;
  }

  /**
   * @brief Puts a robot-reported pose on the map.
   * @details Takes the pose relative to the robot's pose at start (armedPose),
   * turns it by the difference between the home heading and the robot's
   * heading at start, and adds the home pose. With the robot on the home mark,
   * its odometry's "forward" (+x) becomes the home heading on the map.
   * @return A copy with x, y and theta in map coordinates.
   */
  TCPTelemetry_c toMap(TCPTelemetry_c sample) {
    TCPTelemetry_c result = sample.copy();
    float dx = sample.x - armedPose.x;
    float dy = sample.y - armedPose.y;
    float turn = homeTheta - armedPose.theta;
    result.x = homeX + dx * cos(turn) - dy * sin(turn);
    result.y = homeY + dx * sin(turn) + dy * cos(turn);
    result.theta = sample.theta + turn;
    return result;
  }

  /** @brief Clears buffered TCP state and disarms the live-comparison run. */
  void resetLiveSession() {
    tcpQueue.clear();
    liveArmed = false;
    liveRunning = false;
    previousTcpSignal = 0;
    lastTcpTimestampMs = -1;
    ghost = new TCPTelemetry_c();
    if (simulatorMode == LIVE_COMPARISON) controller.reset();
  }

  /** @brief Starts a short asynchronous connection attempt to the StampC3. */
  void beginTcpConnectionAttempt() {
    lastTcpAttemptMs = millis();
    tcpAttemptInProgress = true;
    tcpStatus = "TCP: connecting to " + tcpHost + ":" + tcpPort;
    new Thread(new Runnable() {
      public void run() {
        connectTcpInBackground();
      }
    }).start();
  }

  void connectTcpInBackground() {
    Socket socket = new Socket();
    try {
      socket.connect(new InetSocketAddress(tcpHost, tcpPort), tcpConnectTimeoutMs);
      pendingTcpSocket = socket;
    }
    catch (Exception error) {
      closeSocket(socket);
      tcpAttemptFailed = true;
    }
  }

  void closeSocket(Socket socket) {
    try {
      socket.close();
    }
    catch (Exception ignored) {
    }
  }
}
