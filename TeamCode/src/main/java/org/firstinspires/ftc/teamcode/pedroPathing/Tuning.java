package org.firstinspires.ftc.teamcode.pedroPathing;

/**
 * Pedro Pathing 3 AutoTune registrations.
 *
 * <p>AutoTune ({@code com.pedropathing:tuning}) discovers tuners through its {@code TunerScanner}:
 * every {@code public static} no-arg method in this class annotated with {@code
 * @com.pedropathing.tuning.autotune.Tuner(name = "...")} that returns a {@code
 * com.pedropathing.tuning.autotune.Procedure} is registered and shown in the AutoTune web UI.
 *
 * <p>The old {@code MecanumTuner} / {@code PinpointTuner} / {@code ForesightTuner} classes do not
 * exist in {@code tuning:1.0.0}, so there is nothing to register yet — this matches the Pedro 3.0.1
 * Quickstart's starting state. Add {@code @Tuner} methods here once the tuner procedures you want
 * are available.
 */
public final class Tuning {

  private Tuning() {}
}
