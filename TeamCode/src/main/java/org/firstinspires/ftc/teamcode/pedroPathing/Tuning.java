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
 * Pedro Pathing 3 AutoTune registrations. Every {@code @Tuner} method here shows up in the AutoTune
 * web UI at http://192.168.43.1:10158. Run them in order: Mecanum, Pinpoint, Foresight, then Tests.
 * The procedure classes are copied from the Pedro Pathing Quickstart.
 */
public final class Tuning {

  private Tuning() {}

  @Tuner
  public static Procedure mecanumTuner() {
    return new MecanumTuner();
  }

  @Tuner
  public static Procedure pinpointTuner() {
    return new PinpointTuner();
  }

  @Tuner
  public static Procedure foresightTuner() {
    return new ForesightTuner(
        hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
        hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
  }

  @Tuner
  public static Procedure tests() {
    return new Tests(
        hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
        hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
        () -> new Foresight(Constants.foresightConfig));
  }
}
