package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.Constants.currentRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SoftElectronics {
    private static FtcDashboard dash;
    private static Telemetry telemetryA;
    private static IMU imu;

    private RevHubOrientationOnRobot.UsbFacingDirection  usbDirection;
    private RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
    private RevHubOrientationOnRobot orientationOnRobot;

    public SoftElectronics(Telemetry opModeTelemetry) {
        dash = FtcDashboard.getInstance();

        telemetryA = new MultipleTelemetry(opModeTelemetry, dash.getTelemetry());
        telemetryA.update();

        logoDirection = currentRobot.logoDirection;
        usbDirection  = currentRobot.usbDirection;
        orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw();
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public IMU getIMU() {
        return imu;
    }

    public Telemetry getTelemetry() {
        return telemetryA;
    }

    public FtcDashboard getDashboard() {
        return dash;
    }
}
