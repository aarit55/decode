package org.firstinspires.ftc.teamcode.records;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

/** Immutable live inputs fed into the stateless shot solver every control loop. */
public record ShotInputs(
    Pose robotPose, // Live localizer pose (X, Y, Heading in radians)
    Velocity robotVelocity, // Live field-frame localizer velocity (vx, vy in/s; omega rad/s)
    double targetGoalX, // Target goal center X coordinate on field (inches)
    double targetGoalY // Target goal center Y coordinate on field (inches)
    ) {}
