package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


public abstract class GeneralConstants {
    public RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
    public RevHubOrientationOnRobot.UsbFacingDirection usbDirection;

    public static enum colorSensorStates {
        EMPTY,
        OCCUPIED,
    }

    public static double limelightTolerance;
}
