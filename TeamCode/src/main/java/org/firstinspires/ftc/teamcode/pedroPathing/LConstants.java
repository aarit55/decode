package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.localization.Localizers;
import com.pedropathing.util.FollowerConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        FollowerConstants.localizer = Localizers.PINPOINT;

        FollowerConstants.leftX = 1.5729952;
        FollowerConstants.rightX = 0;
        FollowerConstants.strafeY = -4.535451;

        FollowerConstants.pinpointDeviceName = "pinpoint";

        FollowerConstants.orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
    }
}
