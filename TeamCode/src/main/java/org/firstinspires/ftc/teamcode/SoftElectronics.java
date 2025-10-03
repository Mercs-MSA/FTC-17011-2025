package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.Constants.currentRobotConstants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SoftElectronics {
    private static FtcDashboard dash;
    private static Telemetry telemetryA;
    private static IMU imu;

    private RevHubOrientationOnRobot.UsbFacingDirection  usbDirection;
    private RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
    private RevHubOrientationOnRobot orientationOnRobot;

    public SoftElectronics(HardwareMap hardwareMap, Telemetry opModeTelemetry) {
        dash = FtcDashboard.getInstance();

        telemetryA = new MultipleTelemetry(opModeTelemetry, dash.getTelemetry());
        telemetryA.update();


        logoDirection = currentRobotConstants.logoDirection;
        usbDirection  = currentRobotConstants.usbDirection;
        orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public static double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public static void resetYaw() {
        imu.resetYaw();
    }

    public static IMU getIMU() {
        return imu;
    }

    public static Telemetry getTelemetry() {
        return telemetryA;
    }

    public static FtcDashboard getDashboard() {
        return dash;
    }
}
