package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/*
 * README
 * Edit "version" to the designated robot version before pushing code
 * Make sure to extend GeneralConstants.java
 * Place all common variables in GeneralConstants.java
 *
 */
@Config
public class Constants {

    public static enum VERSIONS {
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
    };

    //VERSION SELECTOR; USE THIS TO SWITCH ROBOT VERSION, CHANGE NOTHING ELSE
    public static final VERSIONS version = VERSIONS.V1;




    public static GeneralConstants currentRobotConstants = version.getConstants();


    public static class AlphaConstants extends GeneralConstants {}

    public static class V1Constants extends GeneralConstants {
        public V1Constants() {
            logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
            usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

            limelightTolerance = 2;

            double yawServoForward;
            double pitchServoZero;
        }
    }

    public static class V2Constants extends GeneralConstants {}

    public static class V3Constants extends GeneralConstants {}

}
