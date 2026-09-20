package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomPIDFCoefficients;

public class FConstants {
    static {
        FollowerConstants.mass = 11.15;

        FollowerConstants.headingPIDFCoefficients = new CustomPIDFCoefficients(0.7, 0.0, 0.002, 0.02);

        FollowerConstants.useForesight = true;

        FollowerConstants.centripetalScaling = 0.0;
    }
}
