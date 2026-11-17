// Replays robot logs through the twin's own Controller.pde (symlinked here)
// and checks it sets the same motor PWMs the robot did.
//
//   Processing cli --sketch=$PWD/tools/twin_replay --run [--config=other.json] logs/run_a.csv [...]
//
// Each row's sensor readings, encoder counts and time are handed to the
// controller as if they came from the robot. No physics runs, so any PWM
// difference is a difference in the logic, not in the twin's models.
// Rows where the robot is following (signal 1) are compared.

class ReplayRobot extends Robot_c {
  long ms;
  int left, right;

  ReplayRobot(JSONObject cfg) {
    super(cfg, new PImage(1, 1), 0, 0);
  }

  long getMillis() { return ms; }
  void getSurfaceSensors() { }   // readings are set from the log row
  int getLeftEncoderCount() { return left; }
  int getRightEncoderCount() { return right; }
}

void settings() {
  size(100, 100);
}

void setup() {
  if (args == null || args.length == 0) {
    println("usage: --run <robot log.csv> [...]");
    exit();
    return;
  }
  String cfgPath = "config/lokum.json";
  ArrayList<String> logs = new ArrayList<String>();
  for (String a : args) {
    if (a.startsWith("--config=")) cfgPath = a.substring(9);
    else logs.add(a);
  }
  JSONObject cfg = loadJSONObject(repoFile(cfgPath).getPath());
  for (String path : logs) replay(cfg, path);
  exit();
}

void replay(JSONObject cfg, String path) {
  java.io.File f = repoFile(path);
  Table log = loadTable(f.getPath(), "header,csv");
  if (log == null || log.getRowCount() == 0) {
    println(path + ": can't read");
    return;
  }

  Controller_c controller = new Controller_c(cfg);
  ReplayRobot robot = new ReplayRobot(cfg);

  // Start the twin's odometry where the robot's was on the first row.
  TableRow first = log.getRow(0);
  controller.odometry.reset(first.getFloat("x_mm"), first.getFloat("y_mm"), first.getFloat("theta_rad"));
  controller.odometry.started = true;
  controller.odometry.lastLeft = first.getInt("enc_left");
  controller.odometry.lastRight = first.getInt("enc_right");

  int compared = 0, exact = 0, offByOne = 0, worse = 0, gaps = 0;
  int maxDiff = 0;
  float maxPoseMm = 0, maxThetaRad = 0;
  long prevMs = -1;
  String firstMismatch = null;
  int[] modeRows = new int[4];
  PrintWriter out = createWriter(sketchPath("../../logs/replay_" + f.getName()));
  out.println("t_ms,signal,mode,robot_left,robot_right,twin_left,twin_right");

  for (TableRow r : log.rows()) {
    robot.ms = r.getLong("t_ms");
    robot.left = r.getInt("enc_left");
    robot.right = r.getInt("enc_right");
    for (int i = 0; i < 5; i++) robot.surface.reading[i] = r.getInt("dn" + (i + 1));
    if (prevMs >= 0 && robot.ms - prevMs != 10) gaps++;
    prevMs = robot.ms;

    // The robot starts or stops before running the follower in the same cycle.
    int signal = r.getInt("signal");
    if (signal != controller.getSignal()) controller.setSignal(signal);
    controller.update(robot);

    float dx = controller.odometry.pose.x - r.getFloat("x_mm");
    float dy = controller.odometry.pose.y - r.getFloat("y_mm");
    maxPoseMm = max(maxPoseMm, sqrt(dx * dx + dy * dy));
    maxThetaRad = max(maxThetaRad, abs(controller.odometry.pose.theta - r.getFloat("theta_rad")));

    if (signal != 1) continue;
    int rl = r.getInt("pwm_left"), rr = r.getInt("pwm_right");
    int tl = robot.motors.left.pwm, tr = robot.motors.right.pwm;
    out.println(robot.ms + "," + signal + "," + controller.mode + "," + rl + "," + rr + "," + tl + "," + tr);
    compared++;
    modeRows[controller.mode]++;
    int d = max(abs(rl - tl), abs(rr - tr));
    maxDiff = max(maxDiff, d);
    if (d == 0) exact++;
    else if (d == 1) offByOne++;
    else {
      worse++;
      if (firstMismatch == null) {
        firstMismatch = "t " + robot.ms + " ms, mode " + controller.mode + ": robot " + rl + "," + rr
          + ", twin " + tl + "," + tr;
      }
    }
  }
  out.flush();
  out.close();

  println(f.getName() + ": " + log.getRowCount() + " rows, " + gaps + " not 10 ms apart");
  println("  following rows compared: " + compared + " (following " + modeRows[1] + ", searching "
    + modeRows[2] + ", stopped " + modeRows[3] + ")");
  if (compared > 0) {
    println(String.format("  PWM exact %d (%.2f %%), off by 1: %d, off by more: %d, max diff %d",
      exact, 100.0 * exact / compared, offByOne, worse, maxDiff));
  }
  if (firstMismatch != null) println("  first bigger mismatch: " + firstMismatch);
  println(String.format("  odometry: max position diff %.3f mm, max heading diff %.5f rad", maxPoseMm, maxThetaRad));
}

// Paths are relative to the repo root unless absolute.
java.io.File repoFile(String path) {
  java.io.File f = new java.io.File(path);
  return f.isAbsolute() ? f : new java.io.File(sketchPath("../.."), path);
}
