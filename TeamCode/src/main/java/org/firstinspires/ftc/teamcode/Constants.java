package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.teamcode.GeneralConstants.*;

/*
 * README
 * Edit "currentRobot" to the designated robot constants before pushing code
 * Make sure to extend GeneralConstants.java
 * Place all common variables in GeneralConstants.java
 *
 */
@Config
public class Constants {

    public static V1Constants currentRobot = new V1Constants();

    public static class AlphaConstants extends GeneralConstants {}

    public static class V1Constants extends GeneralConstants {
        public V1Constants() {
            logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
            usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        }
    }

    public static class V2Constants extends GeneralConstants {}

    public static class V3Constants extends GeneralConstants {}

}
