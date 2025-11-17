package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.Constants.Constants.onBlueAlliance;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivebase {

    // Drive motors
    private static DcMotor frontLeft, frontRight, backLeft, backRight;

    // Odometry / IMU
    private SparkFunOTOS otos;
    private double offset = 0;
    private double IMUheadingTracker = 0;

    // Limelight
    private Limelight3A limelight;
    private LLStatus llStatus;
    private LLResult llResults;

    private double TX = 0;

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

        // Brake when zero power
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // OTOS setup
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setAngularUnit(AngleUnit.RADIANS);
        SparkFunOTOS.Pose2D offsetPose = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offsetPose);
        otos.setAngularScalar(.985889);
        otos.resetTracking();
        otos.calibrateImu();

        // Limelight setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(onBlueAlliance ? 0 : 1);
    }

    public void offsetYaw(double offsetDeg) {
        this.offset = Math.toRadians(offsetDeg);
    }

    public void resetYaw() {
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
    }

    public void setPosition(SparkFunOTOS.Pose2D pose) {
        otos.setPosition(pose);
    }

    public void updateLL() {
        llStatus = limelight.getStatus();
        llResults = limelight.getLatestResult();

        if (llResults != null && llResults.isValid()) {
            double captureLatency = llResults.getCaptureLatency();
            double targetingLatency = llResults.getTargetingLatency();
            double parseLatency = llResults.getParseLatency();
            // log if you want
        }
    }

    // Field-centric drive
    public void drive(double drive, double strafe, double turn) {
        // Get current heading (radians)
        double botHeading = otos.getPosition().h + offset;
        IMUheadingTracker = botHeading;

        // Rotate joystick input to be field-centric
        double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1.0);
        double frontLeftPower  = (rotY + rotX + turn) / denominator;
        double backLeftPower   = (rotY - rotX + turn) / denominator;
        double frontRightPower = (rotY - rotX - turn) / denominator;
        double backRightPower  = (rotY + rotX - turn) / denominator;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
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

    public double getBotHeading() {
        return IMUheadingTracker;
    }

    public boolean getTXInRange() {
        return llResults != null
                && llResults.isValid()
                && !llResults.getFiducialResults().isEmpty()
                && llResults.getFiducialResults().get(0).getTargetXDegrees() <= 0.5
                && llResults.getFiducialResults().get(0).getTargetXDegrees() >= -0.5;
    }

    public boolean getTargetSeen() {
        return llResults != null && llResults.isValid();
    }

    public double getOffset() {
        return offset;
    }

    // Simple proportional turn to an approximate goal heading (67 deg)
    public void turnToGoal() {
        double heading = Math.toDegrees(otos.getPosition().h);
        if (heading > 62 && heading < 72) {
            drive(0, 0, 0);
        } else {
            drive(0, 0, -0.05 * (67 - heading));
        }
    }
}