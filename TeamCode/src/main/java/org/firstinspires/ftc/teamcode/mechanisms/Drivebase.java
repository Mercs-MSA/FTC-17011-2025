package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SoftElectronics;


public class Drivebase {

    // Declare motors
    private static DcMotor frontLeft, frontRight, backLeft, backRight;
    // Constructor
    public Drivebase(HardwareMap hardwareMap) {
        // Initialize motors
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Motor directions (adjust if your robot moves backwards/sideways)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // Field-centric drive
<<<<<<< HEAD
    public void drive(double drive, double strafe, double turn, boolean canDrive) {
        // Get current heading
        double botHeading = SoftElectronics.getYaw();
=======
    public void drive(double drive, double strafe, double turn) {
//        // Get current heading
//        double botHeading = SoftElectronics.getYaw();
//
//        // Rotate joystick input to be field-centric
//        double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
//        double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);
//
//        double flPower = rotY + rotX + turn;
//        double frPower = rotY - rotX - turn;
//        double blPower = rotY - rotX + turn;
//        double brPower = rotY + rotX - turn;
//
//        // Normalize powers
//        double max = Math.max(1.0, Math.max(Math.abs(flPower),
//                Math.max(Math.abs(frPower),
//                        Math.max(Math.abs(blPower), Math.abs(brPower)))));
//
//        frontLeft.setPower(flPower / max);
//        frontRight.setPower(frPower / max);
//        backLeft.setPower(blPower / max);
//        backRight.setPower(brPower / max);
>>>>>>> a52d9ef7e3fd0cfbecafff127a414b7fe76622b2

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double frontLeftPower  = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower   = drive - strafe + turn;
        double backRightPower  = drive + strafe - turn;


        // Send calculated power to wheels
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

<<<<<<< HEAD
        if (canDrive) {
            frontLeft.setPower(flPower / max);
            frontRight.setPower(frPower / max);
            backLeft.setPower(blPower / max);
            backRight.setPower(brPower / max);
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
=======
    public void setFrontLeftPower(double power) {
        frontLeft.setPower(power);
    }
    public void setBackLeftPower(double power) {
        backLeft.setPower(power);
    }
    public void setFrontRightPower(double power) {
        frontRight.setPower(power);
    }
    public void setBackRightPower(double power) {
        backRight.setPower(power);
>>>>>>> a52d9ef7e3fd0cfbecafff127a414b7fe76622b2
    }

    // Stop all motors
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
