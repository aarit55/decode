package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.mecanum;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.stopRobot;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.Paths;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

/**
 * Pedro Pathing 3 tuning menu — a port of the Pedro 2 {@code Tuning} OpMode this project used
 * before the 3.0.1 migration.
 *
 * <p>The measurement tuners (velocity, zero-power deceleration, predictive braking, localization)
 * measure the same physical quantities as before and print the Pedro 3 field each result belongs in
 * ({@link Constants#foresightConfig} / {@link Constants#localizerConfig}). Pedro 2's manual PIDF
 * tuners (translational / heading / drive / centripetal) have no Pedro 3 equivalent, because
 * Foresight replaces those PIDFs; the "Hold Test" and the path tests exercise Foresight's
 * controllers instead. Swerve / analog tuners were dropped (this robot is mecanum).
 *
 * <p>Tuners that need exact control of the motors (velocity, deceleration, braking) drive the
 * {@link Mecanum} directly and only update the localizer, so Foresight never fights them.
 */
@Configurable
@TeleOp(name = "Tuning", group = "Pedro Pathing")
public class Tuning extends SelectableOpMode {
  // Set in onSelect(), before any tuner OpMode runs.
  @SuppressWarnings("NullAway.Init")
  static Follower follower;

  @SuppressWarnings("NullAway.Init")
  static Mecanum mecanum;

  @SuppressWarnings("NullAway.Init")
  static TelemetryManager telemetryM;

  public Tuning() {
    super(
        "Select a Tuning OpMode",
        s -> {
          s.folder(
              "Localization",
              l -> {
                l.add("Localization Test", LocalizationTest::new);
                l.add("Offsets Tuner", OffsetsTuner::new);
                l.add("Forward Tuner", ForwardTuner::new);
                l.add("Lateral Tuner", LateralTuner::new);
                l.add("Turn Tuner", TurnTuner::new);
              });
          s.folder(
              "Automatic",
              a -> {
                a.add("Forward Velocity Tuner", ForwardVelocityTuner::new);
                a.add("Lateral Velocity Tuner", LateralVelocityTuner::new);
                a.add("Forward Zero Power Deceleration Tuner", ForwardZeroPowerDecelTuner::new);
                a.add("Lateral Zero Power Deceleration Tuner", LateralZeroPowerDecelTuner::new);
                a.add("Forward Predictive Braking Tuner", ForwardBrakingTuner::new);
                a.add("Lateral Predictive Braking Tuner", LateralBrakingTuner::new);
              });
          s.folder(
              "Tests",
              t -> {
                t.add("Hold Test", HoldTest::new);
                t.add("Line", Line::new);
                t.add("Triangle", Triangle::new);
                t.add("Circle", Circle::new);
              });
        });
  }

  @Override
  public void onSelect() {
    follower = Constants.createFollower(hardwareMap);
    mecanum = (Mecanum) follower.drivetrain;
    PanelsConfigurables.INSTANCE.refreshClass(this);
    follower.setPose(new Pose(72, 72, 0));
    Drawing.clearHistory();
    telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
  }

  @Override
  public void onLog(List<String> lines) {}

  static void drawCurrent() {
    Drawing.drawRobot(follower.pose());
    Drawing.sendPacket();
  }

  static void drawCurrentAndHistory() {
    Drawing.recordAndDrawHistory(follower.pose());
    drawCurrent();
  }

  /** Full stop with the brakes on, like Pedro 2's {@code stopRobot()}. */
  static void stopRobot() {
    follower.stop();
    mecanum.drive(DrivePowers.zero(), true);
  }

  /** Robot-frame velocity from the localizer: x = forward, y = strafe (left positive). */
  static Vector2D robotVelocity() {
    return follower
        .localizer
        .velocity()
        .toVector2D()
        .toBodyFrame(follower.localizer.pose().heading());
  }

  static double average(List<Double> values) {
    double sum = 0;
    for (double v : values) {
      sum += v;
    }
    return values.isEmpty() ? 0 : sum / values.size();
  }
}

/** Drive around on gamepad 1 and watch the pose; use this first to check localization. */
class LocalizationTest extends OpMode {
  @Override
  public void init() {
    follower.setPose(new Pose(72, 72, 0));
  }

  @Override
  public void init_loop() {
    telemetryM.debug(
        "This prints the robot's position while you drive it with gamepad 1 (robot-centric).");
    telemetryM.update(telemetry);
    follower.update();
    drawCurrent();
  }

  @Override
  public void start() {
    OpModeUtil.startTeleOpDrive(follower);
  }

  @Override
  public void loop() {
    OpModeUtil.setTeleOpDrive(
        follower, -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
    follower.update();

    Pose p = follower.pose();
    telemetryM.debug("x: " + p.x());
    telemetryM.debug("y: " + p.y());
    telemetryM.debug("heading (deg): " + Math.toDegrees(p.heading()));
    telemetryM.update(telemetry);
    drawCurrentAndHistory();
  }
}

/** Accumulates heading across wrap-arounds, replacing Pedro 2's {@code getTotalHeading()}. */
final class TotalHeading {
  private double previous = Double.NaN;
  private double total = 0;

  double update(double heading) {
    if (!Double.isNaN(previous)) {
      total += AngleUnit.normalizeRadians(heading - previous);
    }
    previous = heading;
    return total;
  }
}

/**
 * Turn the robot 180 degrees by hand around its center. Prints the Pinpoint pod offsets (set them
 * to 0 in {@link Constants#localizerConfig} before running).
 */
class OffsetsTuner extends OpMode {
  private final TotalHeading totalHeading = new TotalHeading();

  @Override
  public void init() {
    follower.setPose(new Pose(72, 72, 0));
    follower.update();
    drawCurrent();
  }

  @Override
  public void init_loop() {
    telemetryM.debug(
        "Prerequisite: set xPodOffset and yPodOffset to 0 in Constants.localizerConfig.");
    telemetryM.debug("Turn your robot 180 degrees. Your offsets in inches will be shown.");
    telemetryM.update(telemetry);
    drawCurrent();
  }

  @Override
  public void loop() {
    follower.update();
    Pose p = follower.pose();
    telemetryM.debug("Total angle (deg): " + Math.toDegrees(totalHeading.update(p.heading())));
    telemetryM.debug("Offsets to put in Constants.localizerConfig:");
    telemetryM.debug("xPodOffset (was forwardPodY): " + ((72.0 - p.y()) / 2.0));
    telemetryM.debug("yPodOffset (was strafePodX): " + ((72.0 - p.x()) / 2.0));
    telemetryM.update(telemetry);
    drawCurrentAndHistory();
  }
}

/**
 * Push the robot forward exactly {@code DISTANCE} inches along a ruler. The Pinpoint has no
 * multipliers in Pedro 3, so this reports the measured distance and error: a big error means the
 * pod type / ticksPerUnit in {@link Constants#localizerConfig} is wrong.
 */
@Configurable
class ForwardTuner extends OpMode {
  public static double DISTANCE = 48;

  @Override
  public void init() {
    follower.setPose(new Pose(72, 72, 0));
    follower.update();
    drawCurrent();
  }

  @Override
  public void init_loop() {
    telemetryM.debug("Push your robot forward " + DISTANCE + " inches along a ruler.");
    telemetryM.update(telemetry);
    drawCurrent();
  }

  @Override
  public void loop() {
    follower.update();
    double moved = follower.pose().x() - 72;
    telemetryM.debug("Distance moved: " + moved);
    telemetryM.debug("Expected: " + DISTANCE);
    telemetryM.debug(
        String.format(Locale.ROOT, "Error: %.2f%%", 100.0 * (moved - DISTANCE) / DISTANCE));
    telemetryM.debug("Scale factor (expected / measured): " + (DISTANCE / moved));
    telemetryM.update(telemetry);
    drawCurrentAndHistory();
  }
}

/** Same as {@link ForwardTuner}, but push the robot left. */
@Configurable
class LateralTuner extends OpMode {
  public static double DISTANCE = 48;

  @Override
  public void init() {
    follower.setPose(new Pose(72, 72, 0));
    follower.update();
    drawCurrent();
  }

  @Override
  public void init_loop() {
    telemetryM.debug("Push your robot to the left " + DISTANCE + " inches along a ruler.");
    telemetryM.update(telemetry);
    drawCurrent();
  }

  @Override
  public void loop() {
    follower.update();
    double moved = follower.pose().y() - 72;
    telemetryM.debug("Distance moved: " + moved);
    telemetryM.debug("Expected: " + DISTANCE);
    telemetryM.debug(
        String.format(Locale.ROOT, "Error: %.2f%%", 100.0 * (moved - DISTANCE) / DISTANCE));
    telemetryM.debug("Scale factor (expected / measured): " + (DISTANCE / moved));
    telemetryM.update(telemetry);
    drawCurrentAndHistory();
  }
}

/** Turn the robot {@code ANGLE_DEG} degrees by hand and compare with the measured total angle. */
@Configurable
class TurnTuner extends OpMode {
  public static double ANGLE_DEG = 360;
  private final TotalHeading totalHeading = new TotalHeading();

  @Override
  public void init() {
    follower.setPose(new Pose(72, 72, 0));
    follower.update();
    drawCurrent();
  }

  @Override
  public void init_loop() {
    telemetryM.debug("Turn your robot " + ANGLE_DEG + " degrees counter-clockwise by hand.");
    telemetryM.update(telemetry);
    drawCurrent();
  }

  @Override
  public void loop() {
    follower.update();
    double total = Math.toDegrees(totalHeading.update(follower.pose().heading()));
    telemetryM.debug("Total angle (deg): " + total);
    telemetryM.debug("Scale factor (expected / measured): " + (ANGLE_DEG / total));
    telemetryM.update(telemetry);
    drawCurrentAndHistory();
  }
}

/**
 * Drives at full power for {@code DISTANCE} inches and averages the last {@code RECORD_NUMBER}
 * velocities. Result goes in {@code foresightConfig.maxAchievableForwardVelocity} (Pedro 2's
 * xVelocity) or {@code maxAchievableStrafeVelocity} (Pedro 2's yVelocity).
 */
abstract class VelocityTuner extends OpMode {
  private final ArrayDeque<Double> velocities = new ArrayDeque<>();
  private boolean end;

  abstract boolean forwardAxis();

  abstract double distance();

  abstract int recordNumber();

  @Override
  public void init() {
    follower.setPose(new Pose(72, 72, 0));
  }

  @Override
  public void init_loop() {
    String dir = forwardAxis() ? "forward" : "to the left";
    telemetryM.debug("The robot will run at full power " + dir + " for " + distance() + " inches.");
    telemetryM.debug("Make sure you have enough room: it coasts after cutting power.");
    telemetryM.debug("Press B on gamepad 1 to stop.");
    telemetryM.update(telemetry);
    follower.update();
    drawCurrent();
  }

  @Override
  public void start() {
    velocities.clear();
    end = false;
  }

  @Override
  public void loop() {
    if (gamepad1.bWasPressed()) {
      stopRobot();
      requestOpModeStop();
      return;
    }

    follower.localizer.update();
    drawCurrentAndHistory();
    Pose p = follower.localizer.pose();

    if (!end) {
      double travelled = forwardAxis() ? p.x() - 72 : p.y() - 72;
      if (Math.abs(travelled) > distance()) {
        end = true;
        stopRobot();
      } else {
        mecanum.drive(forwardAxis() ? new DrivePowers(1, 0, 0) : new DrivePowers(0, 1, 0), true);
        Vector2D v = Tuning.robotVelocity();
        velocities.addLast(Math.abs(forwardAxis() ? v.x() : v.y()));
        while (velocities.size() > recordNumber()) {
          velocities.removeFirst();
        }
      }
    } else {
      stopRobot();
      double avg = Tuning.average(new ArrayList<>(velocities));
      String field = forwardAxis() ? "maxAchievableForwardVelocity" : "maxAchievableStrafeVelocity";
      telemetryM.debug((forwardAxis() ? "Forward" : "Lateral") + " velocity: " + avg);
      telemetryM.debug("Put it in Constants.java:  c." + field + ".set(" + avg + ");");
      telemetryM.debug("Press A to use it now (until the robot restarts).");
      telemetryM.update(telemetry);

      if (gamepad1.aWasPressed()) {
        if (forwardAxis()) {
          Constants.foresightConfig.maxAchievableForwardVelocity.set(avg);
        } else {
          Constants.foresightConfig.maxAchievableStrafeVelocity.set(avg);
        }
      }
    }
  }
}

@Configurable
class ForwardVelocityTuner extends VelocityTuner {
  public static double DISTANCE = 48;
  public static int RECORD_NUMBER = 10;

  @Override
  boolean forwardAxis() {
    return true;
  }

  @Override
  double distance() {
    return DISTANCE;
  }

  @Override
  int recordNumber() {
    return RECORD_NUMBER;
  }
}

@Configurable
class LateralVelocityTuner extends VelocityTuner {
  public static double DISTANCE = 48;
  public static int RECORD_NUMBER = 10;

  @Override
  boolean forwardAxis() {
    return false;
  }

  @Override
  double distance() {
    return DISTANCE;
  }

  @Override
  int recordNumber() {
    return RECORD_NUMBER;
  }
}

/**
 * Speeds up to {@code VELOCITY} in/s, cuts power with the motors floating, and averages the
 * deceleration until the robot is nearly stopped. Pedro 2 called this "zero power acceleration" (a
 * negative number); Foresight's {@code naturalForwardDeceleration} / {@code
 * naturalStrafeDeceleration} take the positive magnitude.
 */
abstract class ZeroPowerDecelTuner extends OpMode {
  private final ArrayList<Double> decelerations = new ArrayList<>();
  private double previousVelocity;
  private long previousTimeNano;
  private boolean stopping;
  private boolean end;

  abstract boolean forwardAxis();

  abstract double targetVelocity();

  abstract double endVelocity();

  @Override
  public void init() {
    follower.setPose(new Pose(72, 72, 0));
  }

  @Override
  public void init_loop() {
    String dir = forwardAxis() ? "forward" : "to the left";
    telemetryM.debug(
        "The robot will run " + dir + " until it reaches " + targetVelocity() + " in/s.");
    telemetryM.debug("Then it cuts power and rolls to a stop. Make sure you have enough room.");
    telemetryM.debug("Press B on gamepad 1 to stop.");
    telemetryM.update(telemetry);
    follower.update();
    drawCurrent();
  }

  @Override
  public void start() {
    decelerations.clear();
    stopping = false;
    end = false;
    // manual = false -> motors FLOAT, same as Pedro 2's startTeleopDrive(false).
    mecanum.drive(forwardAxis() ? new DrivePowers(1, 0, 0) : new DrivePowers(0, 1, 0), false);
  }

  @Override
  public void loop() {
    if (gamepad1.bWasPressed()) {
      stopRobot();
      requestOpModeStop();
      return;
    }

    follower.localizer.update();
    drawCurrentAndHistory();

    Vector2D v = Tuning.robotVelocity();
    double velocity = Math.abs(forwardAxis() ? v.x() : v.y());

    if (!end) {
      if (!stopping) {
        mecanum.drive(forwardAxis() ? new DrivePowers(1, 0, 0) : new DrivePowers(0, 1, 0), false);
        if (velocity > targetVelocity()) {
          previousVelocity = velocity;
          previousTimeNano = System.nanoTime();
          stopping = true;
          mecanum.drive(DrivePowers.zero(), false);
        }
      } else {
        long now = System.nanoTime();
        double dt = (now - previousTimeNano) / 1e9;
        if (dt > 0) {
          decelerations.add((previousVelocity - velocity) / dt);
        }
        previousVelocity = velocity;
        previousTimeNano = now;
        if (velocity < endVelocity()) {
          end = true;
        }
      }
    } else {
      double avg = Tuning.average(decelerations);
      String field = forwardAxis() ? "naturalForwardDeceleration" : "naturalStrafeDeceleration";
      telemetryM.debug((forwardAxis() ? "Forward" : "Lateral") + " natural deceleration: " + avg);
      telemetryM.debug("Put it in Constants.java:  c." + field + ".set(" + avg + ");");
      telemetryM.debug("Press A to use it now (until the robot restarts).");
      telemetryM.update(telemetry);

      if (gamepad1.aWasPressed()) {
        if (forwardAxis()) {
          Constants.foresightConfig.naturalForwardDeceleration.set(avg);
        } else {
          Constants.foresightConfig.naturalStrafeDeceleration.set(avg);
        }
      }
    }
  }
}

@Configurable
class ForwardZeroPowerDecelTuner extends ZeroPowerDecelTuner {
  public static double VELOCITY = 30;
  public static double END_VELOCITY = 1;

  @Override
  boolean forwardAxis() {
    return true;
  }

  @Override
  double targetVelocity() {
    return VELOCITY;
  }

  @Override
  double endVelocity() {
    return END_VELOCITY;
  }
}

@Configurable
class LateralZeroPowerDecelTuner extends ZeroPowerDecelTuner {
  public static double VELOCITY = 30;
  public static double END_VELOCITY = 1;

  @Override
  boolean forwardAxis() {
    return false;
  }

  @Override
  double targetVelocity() {
    return VELOCITY;
  }

  @Override
  double endVelocity() {
    return END_VELOCITY;
  }
}

/**
 * Pedro 2's Predictive Braking Tuner. Drives back and forth at decreasing powers, brakes with a
 * small reverse power, and fits braking distance = kLinear * v + kQuadratic * v^2 (no intercept).
 *
 * <p>Foresight uses the same model per axis (brake displacement = quadratic * v|v| + linear * v),
 * so the forward results go in entry (0,0) and the lateral results in entry (1,1) of {@code
 * foresightConfig.linearBrakeCoefficients} / {@code quadraticBrakeCoefficients}. The forward
 * results are also what {@link Constants#BRAKING_K_LINEAR} / {@link Constants#BRAKING_K_QUADRATIC}
 * (Casablanca's zone braking) use.
 */
abstract class BrakingTuner extends OpMode {
  private static final double[] TEST_POWERS = {
    1, 1, 1, 0.9, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2
  };
  private static final double BRAKING_POWER = -0.2;
  private static final int DRIVE_TIME_MS = 1000;

  private enum State {
    START_MOVE,
    WAIT_DRIVE_TIME,
    BRAKING,
    RECORD,
    DONE
  }

  private State state = State.START_MOVE;
  private final ElapsedTime timer = new ElapsedTime();
  private int iteration = 0;
  private Pose startPose = new Pose(0, 0, 0);
  private double measuredVelocity;
  private final List<double[]> samples = new ArrayList<>();
  private double[] result = new double[] {0, 0};

  abstract boolean forwardAxis();

  private DrivePowers powers(double power) {
    return forwardAxis() ? new DrivePowers(power, 0, 0) : new DrivePowers(0, power, 0);
  }

  private double axisVelocity() {
    Vector2D v = Tuning.robotVelocity();
    return forwardAxis() ? v.x() : v.y();
  }

  @Override
  public void init() {
    follower.setPose(new Pose(72, 72, 0));
  }

  @Override
  public void init_loop() {
    telemetryM.debug(
        "The robot will move back and forth "
            + (forwardAxis() ? "(forward/back)" : "(left/right)")
            + ", slowing down each time.");
    telemetryM.debug("Leave at least 4-5 feet of room. Press B on gamepad 1 to stop.");
    telemetryM.update(telemetry);
    follower.update();
    drawCurrent();
  }

  @Override
  public void start() {
    timer.reset();
  }

  /**
   * Least-squares fit of distance = kLinear * v + kQuadratic * v^2 through the origin, the same fit
   * as Pedro 2's {@code MathFunctions.quadraticFit}. Returns {kLinear, kQuadratic}.
   */
  static double[] fitBraking(List<double[]> velocityToDistance) {
    double[] y = new double[velocityToDistance.size()];
    double[][] x = new double[velocityToDistance.size()][2];
    for (int i = 0; i < y.length; i++) {
      double v = velocityToDistance.get(i)[0];
      x[i][0] = v;
      x[i][1] = v * v;
      y[i] = velocityToDistance.get(i)[1];
    }
    OLSMultipleLinearRegression regression = new OLSMultipleLinearRegression();
    regression.setNoIntercept(true);
    regression.newSampleData(y, x);
    return regression.estimateRegressionParameters();
  }

  @Override
  public void loop() {
    if (gamepad1.b) {
      stopRobot();
      requestOpModeStop();
      return;
    }

    follower.localizer.update();
    drawCurrentAndHistory();
    double direction = (iteration % 2 == 0) ? 1 : -1;

    switch (state) {
      case START_MOVE -> {
        if (iteration >= TEST_POWERS.length) {
          result = fitBraking(samples);
          state = State.DONE;
          break;
        }
        mecanum.drive(powers(TEST_POWERS[iteration] * direction), true);
        timer.reset();
        state = State.WAIT_DRIVE_TIME;
      }
      case WAIT_DRIVE_TIME -> {
        mecanum.drive(powers(TEST_POWERS[iteration] * direction), true);
        if (timer.milliseconds() >= DRIVE_TIME_MS) {
          measuredVelocity = Math.abs(axisVelocity());
          startPose = follower.localizer.pose();
          state = State.BRAKING;
        }
      }
      case BRAKING -> {
        mecanum.drive(powers(BRAKING_POWER * direction), true);
        if (axisVelocity() * direction <= 0) {
          mecanum.drive(DrivePowers.zero(), true);
          state = State.RECORD;
        }
      }
      case RECORD -> {
        double brakingDistance = follower.localizer.pose().distance(startPose);
        samples.add(new double[] {measuredVelocity, brakingDistance});
        iteration++;
        state = State.START_MOVE;
      }
      case DONE -> {
        stopRobot();
        int i = forwardAxis() ? 0 : 1;
        telemetryM.debug("Tuning complete (" + (forwardAxis() ? "forward" : "lateral") + " axis)");
        telemetryM.debug("kLinear: " + result[0]);
        telemetryM.debug("kQuadratic: " + result[1]);
        telemetryM.debug(
            "Foresight: linearBrakeCoefficients("
                + i
                + ","
                + i
                + ") = "
                + result[0]
                + ", quadraticBrakeCoefficients("
                + i
                + ","
                + i
                + ") = "
                + result[1]);
        if (forwardAxis()) {
          telemetryM.debug(
              "Casablanca: Constants.BRAKING_K_LINEAR / BRAKING_K_QUADRATIC use these same values.");
        }
        telemetryM.debug("Press A to use them in Foresight now (until the robot restarts).");
        if (gamepad1.aWasPressed()) {
          Matrix lin = Constants.foresightConfig.linearBrakeCoefficients.get();
          Matrix quad = Constants.foresightConfig.quadraticBrakeCoefficients.get();
          double[] l = {lin.get(0, 0), lin.get(1, 1)};
          double[] q = {quad.get(0, 0), quad.get(1, 1)};
          l[i] = result[0];
          q[i] = result[1];
          Constants.foresightConfig.linearBrakeCoefficients.set(Matrix.diag(l));
          Constants.foresightConfig.quadraticBrakeCoefficients.set(Matrix.diag(q));
        }
      }
    }
    for (int k = 0; k < samples.size(); k++) {
      telemetryM.debug(
          String.format(
              Locale.ROOT, "Test %d: v=%.2f  d=%.2f", k, samples.get(k)[0], samples.get(k)[1]));
    }
    telemetryM.update(telemetry);
  }
}

class ForwardBrakingTuner extends BrakingTuner {
  @Override
  boolean forwardAxis() {
    return true;
  }
}

class LateralBrakingTuner extends BrakingTuner {
  @Override
  boolean forwardAxis() {
    return false;
  }
}

/**
 * Holds the start pose with Foresight. Push the robot sideways / twist it and watch it correct —
 * replaces Pedro 2's Translational and Heading PIDF tuners. Tune the controllers in {@code
 * Constants.foresightConfig} (forwardTranslational, strafeTranslational, headingFeedback).
 */
class HoldTest extends OpMode {
  private final Pose target = new Pose(72, 72, 0);

  @Override
  public void init() {
    follower.setPose(target);
  }

  @Override
  public void init_loop() {
    telemetryM.debug("The robot will hold its position. Push and twist it to test the correction.");
    telemetryM.update(telemetry);
    follower.update();
    drawCurrent();
  }

  @Override
  public void start() {
    follower.hold(target, true);
  }

  @Override
  public void loop() {
    follower.update();
    drawCurrentAndHistory();
    Pose p = follower.pose();
    telemetryM.addData("Error X", target.x() - p.x());
    telemetryM.addData("Error Y", target.y() - p.y());
    telemetryM.addData(
        "Error Heading (deg)",
        Math.toDegrees(AngleUnit.normalizeRadians(target.heading() - p.heading())));
    telemetryM.update(telemetry);
  }
}

/** Drives forward and back along a straight line forever. */
@Configurable
class Line extends OpMode {
  public static double DISTANCE = 40;
  private boolean forward = true;
  private Path forwards = Paths.line(new Pose(72, 72, 0), new Pose(72, 72, 0));
  private Path backwards = forwards;

  @Override
  public void init() {
    follower.setPose(new Pose(72, 72, 0));
  }

  @Override
  public void init_loop() {
    telemetryM.debug(
        "The robot will drive forward and back " + DISTANCE + " inches, continuously.");
    telemetryM.update(telemetry);
    follower.update();
    drawCurrent();
  }

  @Override
  public void start() {
    Pose a = new Pose(72, 72, 0);
    Pose b = new Pose(DISTANCE + 72, 72, 0);
    forwards = Paths.line(a, b).constant(0);
    backwards = Paths.line(b, a).constant(0);
    follower.follow(forwards);
  }

  @Override
  public void loop() {
    follower.update();
    drawCurrentAndHistory();
    if (follower.atParametricEnd()) {
      forward = !forward;
      follower.follow(forward ? forwards : backwards);
    }
    telemetryM.debug("Driving forward?: " + forward);
    telemetryM.update(telemetry);
  }
}

/** Drives a triangle forever, starting on the bottom-middle point. */
class Triangle extends OpMode {
  private final Pose startPose = new Pose(72, 72, Math.toRadians(0));
  private final Pose interPose = new Pose(24 + 72, -24 + 72, Math.toRadians(90));
  private final Pose endPose = new Pose(24 + 72, 24 + 72, Math.toRadians(45));
  private Path triangle = Paths.line(startPose, interPose);

  @Override
  public void init() {
    follower.setPose(startPose);
  }

  @Override
  public void init_loop() {
    telemetryM.debug(
        "This will run in a roughly triangular shape, starting on the bottom-middle point.");
    telemetryM.debug("Make sure you have room to the left, front, and right.");
    telemetryM.update(telemetry);
    follower.update();
    drawCurrent();
  }

  @Override
  public void start() {
    follower.setPose(startPose);
    triangle =
        Paths.path(
            Paths.line(startPose, interPose).linear(startPose, interPose),
            Paths.line(interPose, endPose).linear(interPose, endPose),
            Paths.line(endPose, startPose).linear(endPose, startPose));
    follower.follow(triangle);
  }

  @Override
  public void loop() {
    follower.update();
    drawCurrentAndHistory();
    if (follower.atParametricEnd()) {
      follower.follow(triangle);
    }
  }
}

/** Drives a circle of {@code RADIUS} forever while facing its center. */
@Configurable
class Circle extends OpMode {
  public static double RADIUS = 10;
  private Path circle = Paths.line(new Pose(72, 72, 0), new Pose(72, 72, 0));

  @Override
  public void init() {
    follower.setPose(new Pose(72, 72, 0));
  }

  @Override
  public void init_loop() {
    telemetryM.debug("This will run in a circle of radius " + RADIUS + ", facing the center.");
    telemetryM.debug("Make sure you have room to the left, front, and back.");
    telemetryM.update(telemetry);
    follower.update();
    drawCurrent();
  }

  @Override
  public void start() {
    Vector2D center = Vector2D.cartesian(72, RADIUS + 72);
    circle =
        Paths.path(
            Paths.curve(
                    new Pose(72, 72), new Pose(RADIUS + 72, 72), new Pose(RADIUS + 72, RADIUS + 72))
                .facingPoint(center),
            Paths.curve(
                    new Pose(RADIUS + 72, RADIUS + 72),
                    new Pose(RADIUS + 72, (2 * RADIUS) + 72),
                    new Pose(72, (2 * RADIUS) + 72))
                .facingPoint(center),
            Paths.curve(
                    new Pose(72, (2 * RADIUS) + 72),
                    new Pose(-RADIUS + 72, (2 * RADIUS) + 72),
                    new Pose(-RADIUS + 72, RADIUS + 72))
                .facingPoint(center),
            Paths.curve(
                    new Pose(-RADIUS + 72, RADIUS + 72),
                    new Pose(-RADIUS + 72, 72),
                    new Pose(72, 72))
                .facingPoint(center));
    follower.follow(circle);
  }

  @Override
  public void loop() {
    follower.update();
    drawCurrentAndHistory();
    if (follower.atParametricEnd()) {
      follower.follow(circle);
    }
  }
}

/** Panels field drawing for the tuners (robot + recent pose history). */
final class Drawing {
  static final double ROBOT_RADIUS = 9;
  private static final int HISTORY_LENGTH = 300;
  private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
  private static final Style robotLook = new Style("", "#3F51B5", 0.75);
  private static final Style historyLook = new Style("", "#4CAF50", 0.75);
  private static final ArrayDeque<double[]> history = new ArrayDeque<>();

  static {
    panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
  }

  private Drawing() {}

  static void clearHistory() {
    history.clear();
  }

  static void drawRobot(Pose pose) {
    if (Double.isNaN(pose.x()) || Double.isNaN(pose.y()) || Double.isNaN(pose.heading())) {
      return;
    }
    panelsField.setStyle(robotLook);
    panelsField.moveCursor(pose.x(), pose.y());
    panelsField.circle(ROBOT_RADIUS);

    double dx = Math.cos(pose.heading()) * ROBOT_RADIUS;
    double dy = Math.sin(pose.heading()) * ROBOT_RADIUS;
    panelsField.moveCursor(pose.x() + dx / 2, pose.y() + dy / 2);
    panelsField.line(pose.x() + dx, pose.y() + dy);
  }

  static void recordAndDrawHistory(Pose pose) {
    history.addLast(new double[] {pose.x(), pose.y()});
    while (history.size() > HISTORY_LENGTH) {
      history.removeFirst();
    }
    panelsField.setStyle(historyLook);
    double[] prev = null;
    for (double[] point : history) {
      if (prev != null) {
        panelsField.moveCursor(prev[0], prev[1]);
        panelsField.line(point[0], point[1]);
      }
      prev = point;
    }
  }

  static void sendPacket() {
    panelsField.update();
  }
}
