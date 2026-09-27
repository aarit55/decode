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

            c.xPodOffset.set(1.5729952);
            c.yPodOffset.set(-4.535451);

            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);

            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
          });

  public static ForesightConfig foresightConfig =
      new ForesightConfig(
          c -> {

            /*
             * Start with proportional controllers.
             * Pedro 3 normally uses proportional controllers
             * instead of the old PIDF system.
             */
            c.forwardTranslational.set(Controller.proportional(0.3));

            c.strafeTranslational.set(Controller.proportional(0.3));

            /*
             * Your old heading PIDF was:
             *
             * P = 0.7
             * I = 0
             * D = 0.002
             * F = 0.02
             *
             * Pedro 3's Foresight normally uses a
             * proportional heading controller.
             */
            c.headingFeedback.set(Controller.proportional(0.7));

            /*
             * These values are NOT direct conversions of your
             * old PredictiveBrakingCoefficients.
             *
             * They should eventually come from the Pedro 3
             * Foresight tuner.
             */
            c.maxAchievableForwardVelocity.set(75.64281986);
            c.maxAchievableStrafeVelocity.set(58.9247686);

            /*
             * Temporary starting values.
             * These should be tuned with Foresight before
             * relying on the robot for competition.
             */
            c.naturalForwardDeceleration.set(50.0);
            c.naturalStrafeDeceleration.set(50.0);

            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0, 0.0));

            c.linearBrakeCoefficients.set(Matrix.diag(0.0, 0.0));

            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0, 0.0));
          });

  /*
   * TeleOp heading-lock PIDF used by Casablanca. These are the heading PIDF coefficients this robot
   * ran on Pedro 2 (followerConstants.headingPIDFCoefficients). Pedro 3's Foresight has its own
   * heading controller (foresightConfig.headingFeedback), so these now live here.
   */
  public static final double TELEOP_HEADING_KP = 0.7;
  public static final double TELEOP_HEADING_KI = 0.0;
  public static final double TELEOP_HEADING_KD = 0.002;
  public static final double TELEOP_HEADING_KF = 0.02;

  /*
   * Measured stopping model from Pedro 2 (PredictiveBrakingCoefficients 0.05, kLinearBraking,
   * kQuadraticFriction). Casablanca uses it to slow the robot before the protected zone.
   */
  public static final double BRAKING_K_LINEAR = 0.05872647384322376;
  public static final double BRAKING_K_QUADRATIC = 0.001561731123457261;

  /**
   * Predicted braking distance (inches) from {@code velocity} (in/s) — the same formula as Pedro
   * 2's {@code PredictiveBrakingController.computeBrakingDisplacement}: {@code direction * v^2 *
   * kQuadratic + v * kLinear}.
   */
  public static double brakingDisplacement(double velocity, double direction) {
    return direction * velocity * velocity * BRAKING_K_QUADRATIC + velocity * BRAKING_K_LINEAR;
  }

  public static Follower createFollower(HardwareMap hardwareMap) {
    return new Follower(
        new PinpointLocalizer(hardwareMap, localizerConfig),
        new Mecanum(hardwareMap, drivetrainConfig),
        new Foresight(foresightConfig));
  }

  public static Follower createCachedFollower(HardwareMap hardwareMap) {
    return createFollower(hardwareMap);
  }
}
