package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Pedro Pathing 3 drivetrain, localizer and Foresight configuration. Values here are produced by
 * AutoTune (http://192.168.43.1:10158, tuners registered in {@link Tuning}) — paste its generated
 * Java over the matching config below rather than hand-tuning.
 */
public class Constants {

  public static MecanumConfig drivetrainConfig =
      new MecanumConfig(
          c -> {
            c.frontLeftName.set("leftFront");
            c.backLeftName.set("leftBack");
            c.frontRightName.set("rightFront");
            c.backRightName.set("rightBack");

            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

            c.manualBrakeMode.set(true);
          });

  public static PinpointConfig localizerConfig =
      new PinpointConfig(
          c -> {
            c.name.set("pinpoint");
            c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            c.xPodOffset.set(1.5729952);
            c.yPodOffset.set(-4.535451);
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
          });

  /*
   * NOT YET TUNED. Run the Foresight AutoTuner and replace this whole block with its output before
   * relying on autonomous paths. Until then:
   *  - max velocities and the forward/strafe brake coefficients are this robot's measured values
   *    (velocity tuner + braking tuner), with the strafe braking seeded from the forward axis;
   *  - coast/brake kV, heading braking, translational kP and natural deceleration are the Pedro 3
   *    Quickstart reference values.
   */
  public static ForesightConfig foresightConfig =
      new ForesightConfig(
          c -> {
            c.forwardTranslational.set(
                Controller.piecewise(Controller.proportional(0.1))
                    .put(2.5, Controller.proportional(0.3)));
            c.strafeTranslational.set(
                Controller.piecewise(Controller.proportional(0.1))
                    .put(2.5, Controller.proportional(0.3)));

            c.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
            c.brake.set(Controller.proportionalFeedforward(0.008731598255925491));

            c.headingFeedback.set(Controller.proportional(0.7));
            c.headingBrakeCoefficients.set(
                Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));

            c.linearBrakeCoefficients.set(Matrix.diag(0.05872647384322376, 0.05872647384322376));
            c.quadraticBrakeCoefficients.set(
                Matrix.diag(0.001561731123457261, 0.001561731123457261));

            c.maxAchievableForwardVelocity.set(75.64281986);
            c.maxAchievableStrafeVelocity.set(58.9247686);
            c.naturalForwardDeceleration.set(85.01144677379789);
            c.naturalStrafeDeceleration.set(104.49787535782846);
          });

  /**
   * Builds the match Follower. Pedro 3's {@link Mecanum} already caches motor power writes, and
   * bulk-read caching is cleared once per loop in {@code Robot.update()}.
   */
  public static Follower create(HardwareMap hardwareMap) {
    return new Follower(
        new PinpointLocalizer(hardwareMap, localizerConfig),
        new Mecanum(hardwareMap, drivetrainConfig),
        new Foresight(foresightConfig));
  }
}
