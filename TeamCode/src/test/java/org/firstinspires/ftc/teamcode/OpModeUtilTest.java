package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.Mockito;

public class OpModeUtilTest {

  private static final double EPS = 1e-9;

  @Test
  public void testBlackboardPoseKeys() {
    assertEquals("RED_POSE", OpModeUtil.getBlackboardPoseKey(Alliance.RED));
    assertEquals("BLUE_POSE", OpModeUtil.getBlackboardPoseKey(Alliance.BLUE));
  }

  private static Follower manualFollower(double heading) {
    Follower follower = Mockito.mock(Follower.class);
    Mockito.when(follower.manual()).thenReturn(true);
    Mockito.when(follower.pose()).thenReturn(new Pose(72, 72, heading));
    return follower;
  }

  private static DrivePowers captured(Follower follower) {
    ArgumentCaptor<DrivePowers> captor = ArgumentCaptor.forClass(DrivePowers.class);
    Mockito.verify(follower).manual(captor.capture());
    return captor.getValue();
  }

  @Test
  public void testRobotCentricDrivePassesPowersThrough() {
    Follower follower = manualFollower(Math.PI / 2);
    OpModeUtil.setTeleOpDrive(follower, 0.3, -0.4, 0.5, true);
    DrivePowers p = captured(follower);
    assertEquals(0.3, p.forward(), EPS);
    assertEquals(-0.4, p.strafe(), EPS);
    assertEquals(0.5, p.turn(), EPS);
  }

  @Test
  public void testFieldCentricDriveRotatesByMinusHeading() {
    // Robot facing +Y (heading 90 deg). A field +X command must come out as robot-right, i.e.
    // negative strafe — the same result Pedro 2's setTeleOpDrive(..., false) produced.
    Follower follower = manualFollower(Math.PI / 2);
    OpModeUtil.setTeleOpDrive(follower, 1.0, 0.0, 0.0, false);
    DrivePowers p = captured(follower);
    assertEquals(0.0, p.forward(), EPS);
    assertEquals(-1.0, p.strafe(), EPS);
  }

  @Test
  public void testTranslationMagnitudeCappedAndTurnClamped() {
    Follower follower = manualFollower(0.0);
    OpModeUtil.setTeleOpDrive(follower, 1.0, 1.0, 3.0, true);
    DrivePowers p = captured(follower);
    assertEquals(Math.sqrt(0.5), p.forward(), EPS);
    assertEquals(Math.sqrt(0.5), p.strafe(), EPS);
    assertEquals(1.0, p.turn(), EPS);
  }

  @Test
  public void testDriveIgnoredWhileFollowingOrHolding() {
    Follower follower = Mockito.mock(Follower.class);
    Mockito.when(follower.manual()).thenReturn(false);
    OpModeUtil.setTeleOpDrive(follower, 1.0, 0.0, 0.0, true);
    Mockito.verify(follower, Mockito.never()).manual(Mockito.any(DrivePowers.class));
  }
}
