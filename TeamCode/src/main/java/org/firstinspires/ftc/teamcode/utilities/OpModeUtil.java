package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Turret;

public final class OpModeUtil {

  private OpModeUtil() {}

  public static FieldManager initPanelsField() {
    FieldManager field = PanelsField.INSTANCE.getField();
    field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    return field;
  }

  public static String getBlackboardPoseKey(Alliance alliance) {
    return alliance == Alliance.RED ? "RED_POSE" : "BLUE_POSE";
  }

  public static Pose getSavedPose(Alliance alliance, Pose defaultPose) {
    String key = getBlackboardPoseKey(alliance);
    Pose savedPose = (Pose) OpMode.blackboard.get(key);
    return savedPose != null ? savedPose : defaultPose;
  }

  public static void savePose(Alliance alliance, Pose pose) {
    OpMode.blackboard.put(getBlackboardPoseKey(alliance), pose);
  }

  public static void setupTurretAndShooter(Turret turret, Shooter shooter) {
    shooter.setShooterPIDFCoefficients();
    turret.setHoldAngle(0.0);
    turret.setAimMode(Turret.AimMode.IDLE);
  }

  /**
   * Puts the follower into manual (TeleOp) drive with zero powers. Pedro 3 replacement for Pedro
   * 2's {@code follower.startTeleopDrive()}: it cancels any path or hold, just like before.
   */
  public static void startTeleOpDrive(Follower follower) {
    follower.manual(DrivePowers.zero());
  }

  /**
   * Pedro 3 replacement for Pedro 2's {@code follower.setTeleOpDrive(forward, strafe, turn,
   * robotCentric)}, keeping its exact semantics:
   *
   * <ul>
   *   <li>{@code robotCentric == true}: forward/strafe are robot-frame powers.
   *   <li>{@code robotCentric == false}: forward/strafe are field-frame (x/y) powers and are
   *       rotated into the robot frame by the current heading.
   *   <li>The translation vector is capped at magnitude 1 and turn is clamped to [-1, 1].
   *   <li>It only drives while the follower is in manual mode (i.e. after {@link
   *       #startTeleOpDrive}); while a path or hold is running the call is ignored, as in Pedro 2.
   * </ul>
   */
  public static void setTeleOpDrive(
      Follower follower, double forward, double strafe, double turn, boolean robotCentric) {
    if (!follower.manual()) {
      return;
    }
    double magnitude = Math.hypot(forward, strafe);
    if (magnitude > 1.0) {
      forward /= magnitude;
      strafe /= magnitude;
    }
    turn = Math.clamp(turn, -1.0, 1.0);
    DrivePowers powers =
        robotCentric
            ? new DrivePowers(forward, strafe, turn)
            : ManualDrive.fieldCentric(forward, strafe, turn, follower.pose().heading());
    follower.manual(powers);
  }

  public static void drawRobot(
      FieldManager field, Follower follower, Turret turret, double goalX, double goalY) {
    DrawingUtil.drawRobotOnField(
        field,
        follower.pose().x(),
        follower.pose().y(),
        follower.pose().heading(),
        Math.toRadians(turret.getCurrentTurnAngle()),
        goalX,
        goalY);
  }
}
