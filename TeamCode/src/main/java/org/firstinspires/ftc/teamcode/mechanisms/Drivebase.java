package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.Constants.currentRobot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.IMU;


public class Drivebase {

    // Declare motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // IMU for field-centric control
    private BNO055IMU imu_OLD;
    private IMU imu;

    // Constructor
    public Drivebase(HardwareMap hardwareMap) {
        // Initialize motors
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Motor directions (adjust if your robot moves backwards/sideways)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU
        imu_OLD = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu_OLD.initialize(parameters);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = currentRobot.logoDirection;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = currentRobot.usbDirection;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    // Field-centric drive
    public void drive(double drive, double strafe, double turn) {
        // Get current heading
        double botHeading = imu_OLD.getAngularOrientation().firstAngle;

        // Rotate joystick input to be field-centric
        double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

        double flPower = rotY + rotX + turn;
        double frPower = rotY - rotX - turn;
        double blPower = rotY - rotX + turn;
        double brPower = rotY + rotX - turn;

        // Normalize powers
        double max = Math.max(1.0, Math.max(Math.abs(flPower),
                Math.max(Math.abs(frPower),
                        Math.max(Math.abs(blPower), Math.abs(brPower)))));

        frontLeft.setPower(flPower / max);
        frontRight.setPower(frPower / max);
        backLeft.setPower(blPower / max);
        backRight.setPower(brPower / max);
    }

    // Stop all motors
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // Reset IMU heading if needed
    public void resetHeading() {
        imu_OLD.initialize(new BNO055IMU.Parameters());
    }
}
