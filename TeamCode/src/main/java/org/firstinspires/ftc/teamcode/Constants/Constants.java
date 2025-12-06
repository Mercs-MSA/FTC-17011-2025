package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

/*
 * README
 * - Edit "version" to the designated robot version before pushing code.
 * - Make sure to extend GeneralConstants.java for each new version.
 * - Place all common variables in GeneralConstants.java.
 */
@Config
public class Constants {

    // Global match state
    public static boolean onBlueAlliance = true;
    public static boolean ranAuto       = false;

    public static double currentX = 0;
    public static double currentY = 0;
    public static double currentTheta = 0;
    public static int FAR_SHOT_VELOCITY = 1967;
    public static int CLOSE_SHOT_VELOCITY = 1619;

    public static SparkFunOTOS.Pose2D currentPose;
    public static SparkFunOTOS.Pose2D blueGoal = new SparkFunOTOS.Pose2D(50.654, 42.016, Math.toRadians(48.62));
    public static SparkFunOTOS.Pose2D redGoal = new SparkFunOTOS.Pose2D(47.675, -51.351, Math.toRadians(-53.762));



    // Robot version selector
    public enum VERSIONS {
        V1(new V1Constants()),
        V2(new V2Constants()),
        V3(new V3Constants());

        private final GeneralConstants constants;

        VERSIONS(GeneralConstants constant) {
            this.constants = constant;
        }

        public GeneralConstants getConstants() {
            return constants;
        }
    }

    // VERSION SELECTOR; USE THIS TO SWITCH ROBOT VERSION, CHANGE NOTHING ELSE
    public static final VERSIONS version = VERSIONS.V1;

    // The active constants instance for the selected version
    public static final GeneralConstants currentRobotConstants = version.getConstants();

    // Optional: alpha robot type if you ever need one
    public static class AlphaConstants extends GeneralConstants {}

    public static class V1Constants extends GeneralConstants {
        public V1Constants() {
            // IMU orientation
            logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
            usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

            // Limelight tuning
            limelightTolerance = 2.0;

            // Servo reference angles/positions (fill with real values when you have them)
            yawServoForward = 0.5;   // example value
            pitchServoZero  = 0.5;   // example value
        }
    }

    public static class V2Constants extends GeneralConstants {
        public V2Constants() {
            // Put V2 hardware-specific stuff here when you build that robot
        }
    }

    public static class V3Constants extends GeneralConstants {
        public V3Constants() {
            // Put V3 hardware-specific stuff here when you build that robot
        }
    }
}