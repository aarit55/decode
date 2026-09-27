package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Concise utilities for constructing Pedro Pathing 3 paths. */
public final class PathUtil {

  private PathUtil() {}

  /** Straight line, heading interpolated linearly from start to end over the whole path. */
  public static Path pline(Pose start, Pose end) {
    return Paths.line(start, end).linear(start, end);
  }

  /** Straight line, heading reaches the end heading at {@code headingEndTime} (0..1). */
  public static Path pline(Pose start, Pose end, double headingEndTime) {
    return Paths.line(start, end).linear(start, end, headingEndTime);
  }

  /** Quadratic Bezier through one control point, linear heading over the whole path. */
  public static Path pcurve(Pose start, Pose controlPoint, Pose end) {
    return Paths.curve(start, controlPoint, end).linear(start, end);
  }

  /** Quadratic Bezier, heading reaches the end heading at {@code headingEndTime} (0..1). */
  public static Path pcurve(Pose start, Pose controlPoint, Pose end, double headingEndTime) {
    return Paths.curve(start, controlPoint, end).linear(start, end, headingEndTime);
  }
}
