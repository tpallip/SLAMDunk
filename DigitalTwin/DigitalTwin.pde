import processing.net.*;
import processing.core.PApplet;
import java.net.*;

/** @brief Initial application-window width in screen pixels. */
final int WINDOW_WIDTH = 1000;
/** @brief Initial application-window height in screen pixels. */
final int WINDOW_HEIGHT = 800;
/** @brief Initial surface-image x offset from the world origin, in mm. */
final float SURFACE_OFFSET_X_MM = 135.0;
/** @brief Initial surface-image y offset from the world origin, in mm. */
final float SURFACE_OFFSET_Y_MM = 0.0;

/** @brief Application-level owner of the simulated robot and UI. */
Simulator simulator;

/** @brief Configures Processing's initial drawing surface. */
void settings() {
  size(WINDOW_WIDTH, WINDOW_HEIGHT);
}

/** @brief Creates the resizable simulator window and its application state. */
void setup() {
  
  // Keep a normal window (rather than fullscreen) so students can resize it
  // or place it beside their controller code. Starting at the display size
  // gives the same practical benefit as maximising it at launch.
  surface.setResizable(true);
  
  //surface.setSize(displayWidth, displayHeight);
  //surface.setLocation(0, 0);
  surface.setTitle("CW2026 Digital Twin");
  simulator = new Simulator(this, SURFACE_OFFSET_X_MM, SURFACE_OFFSET_Y_MM);

  // Batch runs with no window updates, then quit:
  // Processing cli --sketch=<path>/DigitalTwin --run batch <runs> <seconds> [cw]
  // cw starts facing the other way round the course (clockwise).
  if (args != null && args.length >= 3 && args[0].equals("batch")) {
    if (args.length == 4 && args[3].equals("cw")) simulator.homeTheta += PI;
    // drop the empty sandbox log the constructor started
    simulator.closeRunLog();
    new java.io.File(sketchPath(simulator.logDir + "/" + simulator.runLogName)).delete();
    simulator.runBatch(int(args[1]), int(args[2]));
    exit();
  }
}

/** @brief Advances the application state and renders one frame. */
void draw() {
  simulator.update();
  simulator.draw();
}

/** @brief Forwards keyboard input to the simulator controls.
 * @see Simulator#handleKey(char, int)
 */
void keyPressed() {
  simulator.handleKey(key, keyCode);
}
