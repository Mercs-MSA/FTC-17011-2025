package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.Constants.Constants.blueGoal;
import static org.firstinspires.ftc.teamcode.Constants.Constants.onBlueAlliance;
import static org.firstinspires.ftc.teamcode.Constants.Constants.redGoal;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


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

    //Odometry
//    private Follower follower;
    public static Pose startingPose = new Pose(88, 8, 90);


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
        SparkFunOTOS.Pose2D offsetPose = new SparkFunOTOS.Pose2D(-0.375, -7.1875, Math.PI);
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, Math.PI));
        otos.setOffset(offsetPose);
        otos.setLinearScalar(1.04);
        otos.setAngularScalar(.9855);
        otos.resetTracking();
        otos.calibrateImu();

        // Limelight setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(onBlueAlliance ? 0 : 1);

        //Odometry setup
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
//        follower.update();
    }

    public void offsetYaw(double offsetDeg) {
        this.offset = Math.toRadians(offsetDeg);
    }

    public void resetYaw() {
        otos.setPosition(new SparkFunOTOS.Pose2D(otos.getPosition().x, otos.getPosition().y, 0));
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
//        follower.update();

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

    public LLResult getLLResult() {
        return limelight.getLatestResult();
    }

    // Simple proportional turn to an approximate goal heading (67 deg)
//    public void turnToGoal() {
//        double heading = Math.toDegrees(otos.getPosition().h);
//        if (heading > 62 && heading < 72) {
//            drive(0, 0, 0);
//        } else {
//            drive(0, 0, -0.05 * (67 - heading));
//        }
//    }

//    public Pose getPose() {
//        return follower.getPose();
//    }

    public double getLaserHeading() {
        return otos.getPosition().h;
    }

//    public void setStartingPose(Pose pose) {
//        startingPose = pose;
//    }

    public double distanceToTarget() {
        double range = 0;
        if (onBlueAlliance) {
            range = Math.hypot(otos.getPosition().x - blueGoal.x, otos.getPosition().y - blueGoal.y);
        } else {
            range = Math.hypot(otos.getPosition().x - redGoal.x, otos.getPosition().y - redGoal.y);
        }
        return range;
    }

    public double angleToTargetDeg() {
        Pose blueGoalPedro = new Pose(17, 37);
        Pose redGoalPedro = new Pose (129, 138);

        Pose goalPose = onBlueAlliance ? blueGoalPedro : redGoalPedro;

        double deltaX = goalPose.getX() - getPosition().x;
        double deltaY = goalPose.getY() - getPosition().y;

        return Math.toDegrees(Math.atan2(deltaY, deltaX));
    }
 }