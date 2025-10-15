package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SoftElectronics;


public class Drivebase {

    // Declare motors
    private static DcMotor frontLeft, frontRight, backLeft, backRight;
    private SparkFunOTOS otos;

    private double TX = 0;

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

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setAngularUnit(AngleUnit.RADIANS);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, Math.PI);
        otos.setOffset(offset);
        otos.setAngularScalar(.98946);
        otos.resetTracking();
        otos.calibrateImu();

        TX = 0;
    }

    // Field-centric drive
    public void drive(double drive, double strafe, double turn) {
        // Get current heading
        double botHeading = SoftElectronics.getYaw();
//        double botHeading = otos.getPosition().h;

        // Rotate joystick input to be field-centric
        double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double frontLeftPower = (rotY + rotX + turn) / denominator;
        double backLeftPower = (rotY - rotX + turn) / denominator;
        double frontRightPower = (rotY - rotX - turn) / denominator;
        double backRightPower = (rotY + rotX - turn) / denominator;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

//         Combine the joystick requests for each axis-motion to determine each wheel's power.
//         Set up a variable for each drive wheel to save the power level for telemetry.
//        double frontLeftPower  = drive + strafe + turn;
//        double frontRightPower = drive - strafe - turn;
//        double backLeftPower   = drive - strafe + turn;
//        double backRightPower  = drive + strafe - turn;
//
//
//        // Send calculated power to wheels
//        frontLeft.setPower(frontLeftPower);
//        frontRight.setPower(frontRightPower);
//        backLeft.setPower(backLeftPower);
//        backRight.setPower(backRightPower);
    }


    public void setPosition(SparkFunOTOS.Pose2D pose) {
        otos.setPosition(pose);;
    }


    public void turnToGoal (boolean onBlueAlliance, LLResult llResult) {
        double botHeading = Math.toDegrees(otos.getPosition().h);
        if (llResult.isValid())
            TX = llResult.getFiducialResults().get(0).getTargetXDegrees();

        if (onBlueAlliance) {
            if (!llResult.isValid()) {
                if (botHeading < 44 && botHeading > -135) {
                    drive(0, 0, -.5 * (Math.abs(45 - botHeading) / 10));
                } else {
                    drive(0, 0, .5 * (Math.abs(45 - botHeading) / 10));
                }
            } else {
                if (TX < -2) {
                    drive(0, 0, -.5 * (Math.abs(TX) / 50));
                } else if (TX > 2) {
                    drive(0, 0, .5 * (Math.abs(TX) / 50));
                }
            }
        } else {
            if (!llResult.isValid()) {
                if (botHeading > -44 && botHeading < 135) {
                    drive(0, 0, .5 * (Math.abs(-45 - botHeading) / 0));
                } else {
                    drive(0, 0, -.5 * (Math.abs(-45 - botHeading) / 10));
                }
            } else {
                if (TX < -2) {
                    drive(0, 0, -.5 * (Math.abs(TX) / 50));
                } else if (TX > 2) {
                    drive(0, 0, .5 * (Math.abs(TX) / 50));
                }
            }
        }
    }


    // Stop all motors
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }


    public SparkFunOTOS.Pose2D getPosition() {
        return otos.getPosition();
    }
}
