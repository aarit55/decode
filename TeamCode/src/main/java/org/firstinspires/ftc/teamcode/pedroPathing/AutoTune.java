package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.Tests;

/**
 * Pedro Pathing 3 AutoTune procedures. AutoTune finds every static {@code @Tuner} method here and
 * lists it in its web UI: connect to the robot's Wi-Fi and open http://192.168.43.1:10158.
 *
 * <p>The procedure classes in {@code procedures/} are copied from the official Pedro Pathing
 * Quickstart (github.com/Pedro-Pathing/Quickstart, TeamCode/.../pedro/procedures).
 */
public final class AutoTune {

  private AutoTune() {}

  /** Finds the name and direction of each drivetrain motor. */
  @Tuner
  public static Procedure mecanumTuner() {
    return new MecanumTuner();
  }

  /** Finds the Pinpoint pod offsets and directions. */
  @Tuner
  public static Procedure pinpointTuner() {
    return new PinpointTuner();
  }

  /** Measures and tunes the Foresight constants (velocities, decelerations, braking). */
  @Tuner
  public static Procedure foresightTuner() {
    return new ForesightTuner(
        hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
        hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
  }

  /** Driving and path tests for the tuned Follower. */
  @Tuner
  public static Procedure tests() {
    return new Tests(
        hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
        hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
        () -> new Foresight(Constants.foresightConfig));
  }
}
