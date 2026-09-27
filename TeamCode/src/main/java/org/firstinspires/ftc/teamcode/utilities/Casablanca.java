package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.locationtech.jts.geom.Envelope;

@Configurable
public class Casablanca {

  public static boolean enableFrictionComp = true;
  public static double frictionX;
  public static double frictionY;
  public static double frictionRot;

  public static boolean enableInputSmoothing = true;
  public static double smoothTime;
  public static boolean extraSmoothBackLift = true;
  public static double backLiftMultiplier;

  public static double wallRepulsionPower;
  public static double decelSafetyFactor;

  public static boolean enableDepthProtection = true;
  public static double depthSlowDown;
  public static double depthHardStop;

  public static boolean enableSideProtection = true;
  public static double sideSlowDown;
  public static double sideHardStop;

  public static double laneBlendDistance;
  public static double rotationLookaheadTimeSeconds;

  public static boolean fieldCentric;
  public static double fieldCentricOffsetRad;

  public static boolean enableHeadingLock;
  public static double headingLockIntentThreshold;
  public static double headingLockKsMoving;
  public static double headingLockMovingSpeedThreshold;
  public static double headingLockMaxPower;
  public static double headingLockErrorDeadbandRad;
  public static double headingLockSettleRateRad;
  public static double headingLockStaticFF;

  private double targetHeading = 0.0;
  private boolean headingLockInitialized = false;
  private final Controller headingController;
  private double headingLockKs = 0.0;

  private final ElapsedTime timer = new ElapsedTime();
  private double currentForward = 0;
  private double currentStrafe = 0;
  private double currentTurn = 0;

  private final Sentinel sentinel;
  private final Foresight foresight;

  private double lastLookaheadRad = 0.0;
  private boolean lastRotationSafe = true;
  private double lastAngularVelocityUsed = 0.0;

  private double lastLaneFadeX = 0.0;
  private double lastLaneFadeY = 0.0;
  private double lastDepthScale = 1.0;
  private double lastSideScale = 1.0;
  private boolean lastDepthRepulsion = false;
  private boolean lastSideRepulsion = false;
  private boolean lastPoseUntrusted = false;
  private Envelope lastRobotBounds;
  private Envelope lastProtectedZone;

  public boolean wasPoseUntrusted() {
    return lastPoseUntrusted;
  }

  public double getLastLaneFadeX() {
    return lastLaneFadeX;
  }

  public double getLastLaneFadeY() {
    return lastLaneFadeY;
  }

  public double getLastDepthScale() {
    return lastDepthScale;
  }

  public double getLastSideScale() {
    return lastSideScale;
  }

  public boolean getLastDepthRepulsion() {
    return lastDepthRepulsion;
  }

  public boolean getLastSideRepulsion() {
    return lastSideRepulsion;
  }

  public Envelope getLastRobotBounds() {
    return lastRobotBounds;
  }

  public Envelope getLastProtectedZone() {
    return lastProtectedZone;
  }

  public double getLastLookaheadRad() {
    return lastLookaheadRad;
  }

  public boolean getLastRotationSafe() {
    return lastRotationSafe;
  }

  public double getLastAngularVelocityUsed() {
    return lastAngularVelocityUsed;
  }

  /**
   * @param foresight the match Follower's Foresight algorithm; its tuned braking model predicts how
   *     far the robot will travel before it can stop.
   */
  public Casablanca(Sentinel sentinel, Foresight foresight) {
    this.sentinel = sentinel;
    this.foresight = foresight;

    var c = config.casablanca;
    frictionX = c.friction.x;
    frictionY = c.friction.y;
    frictionRot = c.friction.rot;

    smoothTime = c.smoothing.time;
    backLiftMultiplier = c.smoothing.back_lift_multiplier;

    wallRepulsionPower = c.repulsion.power;
    decelSafetyFactor = c.repulsion.decel_safety_factor;

    enableDepthProtection = c.enable_depth_protection;
    depthSlowDown = c.depth.slow_down;
    depthHardStop = c.depth.hard_stop;

    enableSideProtection = c.enable_side_protection;
    sideSlowDown = c.side.slow_down;
    sideHardStop = c.side.hard_stop;

    laneBlendDistance = c.lane_blend_distance;
    rotationLookaheadTimeSeconds = config.sentinel.rotation_lookahead_time;

    fieldCentric = config.teleop.field_centric;
    fieldCentricOffsetRad = Math.toRadians(config.teleop.field_centric_offset_deg);

    var hl = c.heading_lock;
    enableHeadingLock = hl.enabled;
    headingLockIntentThreshold = hl.intent_threshold;
    headingLockKsMoving = hl.ks_moving;
    headingLockMovingSpeedThreshold = hl.moving_speed_threshold;
    headingLockMaxPower = hl.max_power;
    headingLockErrorDeadbandRad = Math.toRadians(hl.error_deadband_deg);
    headingLockSettleRateRad = Math.toRadians(hl.settle_rate_dps);
    headingLockStaticFF = hl.static_ff;

    this.headingController =
        Controller.sum(
            Controller.pid(hl.pid.p, hl.pid.i, hl.pid.d),
            Controller.staticFeedforward(() -> headingLockStaticFF + headingLockKs));

    performBrakingSanityCheck();

    reset();
  }

  /**
   * Stopping distance (inches) Foresight predicts from full forward speed, before the safety
   * factor.
   */
  public double maxForwardBrakingDistance() {
    double maxVelX = foresight.config.maxAchievableForwardVelocity.get();
    return Math.abs(foresight.getBrakeDisplacement(new Velocity(maxVelX, 0, 0).toTwist(0), 0).x());
  }

  /**
   * Stopping distance (inches) Foresight predicts from full strafe speed, before the safety factor.
   */
  public double maxStrafeBrakingDistance() {
    double maxVelY = foresight.config.maxAchievableStrafeVelocity.get();
    return Math.abs(foresight.getBrakeDisplacement(new Velocity(0, maxVelY, 0).toTwist(0), 0).y());
  }

  private void performBrakingSanityCheck() {
    double maxVelX = foresight.config.maxAchievableForwardVelocity.get();
    double maxVelY = foresight.config.maxAchievableStrafeVelocity.get();
    double minBrakingX = maxForwardBrakingDistance() / decelSafetyFactor;
    double minBrakingY = maxStrafeBrakingDistance() / decelSafetyFactor;

    com.qualcomm.robotcore.util.RobotLog.ii(
        "Casablanca",
        "Init check: depthHardStop=%.2f in (physics stopping dist at max %.1f in/s is %.2f in), sideHardStop=%.2f in (physics stopping dist at max %.1f in/s is %.2f in)",
        depthHardStop,
        maxVelX,
        minBrakingX,
        sideHardStop,
        maxVelY,
        minBrakingY);
  }

  public final void reset() {
    timer.reset();
    currentForward = 0;
    currentStrafe = 0;
    currentTurn = 0;
    headingLockInitialized = false;
    armedAimActive = false;
    goalLockActive = false;
    if (headingController != null) {
      headingController.reset();
    }
  }

  private double armedAimHeadingTarget = 0.0;
  private boolean armedAimActive = false;

  private double goalLockHeadingTarget = 0.0;
  private boolean goalLockActive = false;

  public void setArmedAimTarget(double targetHeadingRad, boolean armed) {
    this.armedAimHeadingTarget = targetHeadingRad;
    this.armedAimActive = armed;
  }

  public void setGoalHeadingLock(double targetHeadingRad, boolean active) {
    if (active && !goalLockActive) {
      headingController.reset();
    }
    this.goalLockHeadingTarget = targetHeadingRad;
    this.goalLockActive = active;
  }

  public boolean isGoalHeadingLockActive() {
    return goalLockActive;
  }

  public double[] adjustDriveInput(
      Pose pose,
      Vector2D currentVelocity,
      double currentAngularVelocity,
      double strafe,
      double forward,
      double turn) {
    return adjustDriveInput(
        pose, currentVelocity, currentAngularVelocity, strafe, forward, turn, turn);
  }

  public double[] adjustDriveInput(
      Pose pose,
      Vector2D currentVelocity,
      double currentAngularVelocity,
      double strafe,
      double forward,
      double turn,
      double rawTurnIntent) {
    if (!Double.isFinite(currentAngularVelocity)) {
      currentAngularVelocity = 0.0;
    }
    if (currentVelocity == null
        || !Double.isFinite(currentVelocity.x())
        || !Double.isFinite(currentVelocity.y())) {
      currentVelocity = Vector2D.zero();
    }
    boolean poseFinite =
        Double.isFinite(pose.x()) && Double.isFinite(pose.y()) && Double.isFinite(pose.heading());

    lastPoseUntrusted = !poseFinite;
    if (!poseFinite) {
      currentForward = 0;
      currentStrafe = 0;
      currentTurn = 0;
      headingLockInitialized = false;
      return new double[] {0.0, 0.0, 0.0};
    }

    if (fieldCentric) {
      Vector2D stick =
          Vector2D.cartesian(forward, strafe).rotate(fieldCentricOffsetRad - pose.heading());
      forward = stick.x();
      strafe = stick.y();
    }

    if (enableFrictionComp) {
      forward = applyFriction(forward, frictionX);
      strafe = applyFriction(strafe, frictionY);
      turn = applyFriction(turn, frictionRot);
    }

    boolean stickReleased = Math.abs(rawTurnIntent) < headingLockIntentThreshold;

    if (armedAimActive && Double.isFinite(armedAimHeadingTarget)) {
      targetHeading = armedAimHeadingTarget;
      headingLockInitialized = true;
    } else if (goalLockActive && Double.isFinite(goalLockHeadingTarget)) {
      targetHeading = goalLockHeadingTarget;
      headingLockInitialized = true;
    }

    if (armedAimActive || goalLockActive || enableHeadingLock && stickReleased) {
      if (!headingLockInitialized) {
        if (Math.abs(currentAngularVelocity) < headingLockSettleRateRad) {
          targetHeading = pose.heading();
          headingLockInitialized = true;
          headingController.reset();
        }
        turn = 0.0;
      } else {
        double headingError = AngleUnit.normalizeRadians(targetHeading - pose.heading());

        if (Math.abs(headingError) < headingLockErrorDeadbandRad) {
          headingController.reset();
          turn = 0.0;
        } else {

          double speedMag = currentVelocity.magnitude();
          double speedRatio = Math.clamp(speedMag / headingLockMovingSpeedThreshold, 0.0, 1.0);
          headingLockKs = frictionRot + speedRatio * (headingLockKsMoving - frictionRot);

          double correction = headingController.calculate(0.0, headingError);
          turn = Math.clamp(correction, -headingLockMaxPower, headingLockMaxPower);
        }
      }
    } else {
      headingLockInitialized = false;
    }

    if (enableInputSmoothing) {
      double dt = timer.seconds();
      timer.reset();
      if (dt > 0.2) dt = 0.05;

      double maxChange = (1.0 / smoothTime) * dt;
      double forwardDiff = forward - currentForward;
      double allowedForwardChange =
          (extraSmoothBackLift && forwardDiff < 0) ? maxChange / backLiftMultiplier : maxChange;

      currentForward = applySlewLimit(currentForward, forward, allowedForwardChange);
      currentStrafe = applySlewLimit(currentStrafe, strafe, maxChange);
      currentTurn = applySlewLimit(currentTurn, turn, maxChange * 2.5);

      forward = currentForward;
      strafe = currentStrafe;
      turn = currentTurn;
    }

    Vector2D inputField = Vector2D.cartesian(forward, strafe).rotate(pose.heading());
    double adjFieldX = inputField.x();
    double adjFieldY = inputField.y();

    Envelope robotBounds = sentinel.getRobotBounds(pose);
    Envelope protectedZone = sentinel.getProtectedZone();

    double currentVelX = currentVelocity.x();
    double currentVelY = currentVelocity.y();

    // Field-frame distance the robot would cover if it braked now, from Foresight's tuned model.
    Pose brakeDisplacement =
        foresight.getBrakeDisplacement(
            new Velocity(currentVelX, currentVelY, currentAngularVelocity).toTwist(pose.heading()),
            pose.heading());

    double laneFadeY =
        calculateLaneFade(
            robotBounds.getMinY(),
            robotBounds.getMaxY(),
            protectedZone.getMinY(),
            protectedZone.getMaxY(),
            laneBlendDistance);
    double laneFadeX =
        calculateLaneFade(
            robotBounds.getMinX(),
            robotBounds.getMaxX(),
            protectedZone.getMinX(),
            protectedZone.getMaxX(),
            laneBlendDistance);

    lastRobotBounds = robotBounds;
    lastProtectedZone = protectedZone;
    lastLaneFadeX = laneFadeX;
    lastLaneFadeY = laneFadeY;
    lastDepthScale = 1.0;
    lastSideScale = 1.0;
    lastDepthRepulsion = false;
    lastSideRepulsion = false;

    if (enableDepthProtection && laneFadeY > 0) {
      AxisState xState =
          calculateAxisState(
              robotBounds.getMinX(),
              robotBounds.getMaxX(),
              protectedZone.getMinX(),
              protectedZone.getMaxX(),
              currentVelX,
              brakeDisplacement.x(),
              adjFieldX,
              depthSlowDown,
              depthHardStop);

      lastDepthScale = xState.scale;
      adjFieldX *= (1.0 + laneFadeY * (xState.scale - 1.0));
      if (laneFadeY >= 0.9 && xState.triggerRepulsion) {
        adjFieldX = xState.repulsionDir * wallRepulsionPower;
        lastDepthRepulsion = true;
      }
    }

    if (enableSideProtection && laneFadeX > 0) {
      AxisState yState =
          calculateAxisState(
              robotBounds.getMinY(),
              robotBounds.getMaxY(),
              protectedZone.getMinY(),
              protectedZone.getMaxY(),
              currentVelY,
              brakeDisplacement.y(),
              adjFieldY,
              sideSlowDown,
              sideHardStop);

      lastSideScale = yState.scale;
      adjFieldY *= (1.0 + laneFadeX * (yState.scale - 1.0));
      if (laneFadeX >= 0.9 && yState.triggerRepulsion) {
        adjFieldY = yState.repulsionDir * wallRepulsionPower;
        lastSideRepulsion = true;
      }
    }

    double lookaheadRad = Math.abs(currentAngularVelocity) * rotationLookaheadTimeSeconds;
    lastLookaheadRad = lookaheadRad;
    lastAngularVelocityUsed = currentAngularVelocity;
    boolean rotationSafe =
        Double.isFinite(turn)
            && Double.isFinite(lookaheadRad)
            && sentinel.isRotationSafe(pose, turn, lookaheadRad);
    lastRotationSafe = rotationSafe;
    if (turn != 0 && !rotationSafe) {
      turn = 0;
    }

    return new double[] {adjFieldY, adjFieldX, turn};
  }

  public static double applyFriction(double input, double kS) {
    if (Math.abs(input) < 0.01) return 0;
    return Math.signum(input) * (kS + Math.abs(input) * (1.0 - kS));
  }

  public static double applySlewLimit(double current, double target, double maxChange) {
    double diff = target - current;
    if (Math.abs(diff) <= maxChange) return target;
    return current + Math.signum(diff) * maxChange;
  }

  public static double calculateLaneFade(
      double botMin, double botMax, double zoneMin, double zoneMax, double buffer) {
    double distToStrict = Math.max(zoneMin - botMax, botMin - zoneMax);
    if (distToStrict <= 0) return 1.0;
    if (distToStrict >= buffer) return 0.0;
    return 1.0 - (distToStrict / buffer);
  }

  private AxisState calculateAxisState(
      double botMin,
      double botMax,
      double zoneMin,
      double zoneMax,
      double currentVel,
      double brakingDisplacement,
      double inputVel,
      double slowDownDist,
      double hardStopDist) {
    AxisState state = new AxisState();
    double distToStop;
    boolean movingTowards = false;
    double repulsionDir;

    if (botMax <= zoneMin) {
      distToStop = zoneMin - botMax;
      repulsionDir = -1.0;
      if (currentVel > 0 || inputVel > 0) movingTowards = true;
    } else if (botMin >= zoneMax) {
      distToStop = botMin - zoneMax;
      repulsionDir = 1.0;
      if (currentVel < 0 || inputVel < 0) movingTowards = true;
    } else {
      state.triggerRepulsion = true;
      double zoneCenter = (zoneMin + zoneMax) / 2.0;
      double botCenter = (botMin + botMax) / 2.0;
      state.repulsionDir = (botCenter < zoneCenter) ? -1.0 : 1.0;
      state.scale = 0.0;
      return state;
    }

    state.repulsionDir = repulsionDir;

    if (!movingTowards) {
      state.scale = 1.0;
      return state;
    }

    if (distToStop <= hardStopDist) {
      state.scale = 0.0;
      state.triggerRepulsion = true;
      return state;
    }

    double proximityScale = 1.0;
    if (distToStop < slowDownDist) {
      proximityScale = (distToStop - hardStopDist) / (slowDownDist - hardStopDist);
    }

    double physicsScale = 1.0;
    if (Math.abs(currentVel) > 0.2) {
      double brakingRoom = Math.max(0, distToStop - hardStopDist);
      double predictedBrakingDist = Math.abs(brakingDisplacement) / decelSafetyFactor;

      if (predictedBrakingDist > brakingRoom && predictedBrakingDist > 0) {
        physicsScale = Math.sqrt(brakingRoom / predictedBrakingDist);
      }
    }
    state.scale = Math.min(proximityScale, physicsScale);
    return state;
  }

  private static class AxisState {
    double scale = 1.0;
    boolean triggerRepulsion = false;
    double repulsionDir = 0.0;
  }
}
