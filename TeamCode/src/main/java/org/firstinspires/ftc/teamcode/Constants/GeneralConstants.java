package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public abstract class GeneralConstants {

    // IMU mounting configuration
    public RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
    public RevHubOrientationOnRobot.UsbFacingDirection  usbDirection;

    // Simple enum for color sensors (you can keep even if not used right now)
    public enum colorSensorStates {
        EMPTY,
        OCCUPIED,
    }

    // Limelight + aiming
    public double limelightTolerance;

    // Turret / camera servo reference positions (optional but handy to have here)
    public double yawServoForward;
    public double pitchServoZero;
}