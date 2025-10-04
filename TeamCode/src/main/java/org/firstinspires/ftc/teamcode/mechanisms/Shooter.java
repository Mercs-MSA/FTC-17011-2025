package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Shooter {
    private DcMotorEx shooterMotorLeft, shooterMotorRight;
    private ColorRangeSensor exitSensor;
//    private CRServo shooterServoYaw;
//    private Servo shooterServoPitch;

    private Limelight3A limelight;
    private LLStatus llStatus;
    private LLResult llResults;

    private static int goalAngle = 0;
    private static int currentAngle = 0;
    public static int pipeline = 0;

    public int getCurrentAngle() {
        return currentAngle;
    }

    public int getGoalAngle() {
        return goalAngle;
    }

    public void updateLL() {
        llStatus = limelight.getStatus();
        llResults = limelight.getLatestResult();

        if (llResults.isValid()) {
            double captureLatency =llResults.getCaptureLatency();
            double targetingLatency = llResults.getTargetingLatency();
            double parseLatency = llResults.getParseLatency();


        }


    }


    public Shooter(HardwareMap hardwareMap) {
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "shooterMotorRight");
//        shooterServoYaw = hardwareMap.get(CRServo.class, "shooterServoYaw");
//        shooterServoPitch = hardwareMap.get(Servo.class, "shooterServoPitch");
        exitSensor = hardwareMap.get(ColorRangeSensor.class, "exitSensor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.start();
        limelight.pipelineSwitch(0); //TODO: Edit pipelines to filter out tags in Limelight Dash

        // Configure initial settings
        shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotorRight.setDirection(DcMotor.Direction.REVERSE);
        shooterMotorLeft.setDirection(DcMotor.Direction.FORWARD);

        shooterMotorLeft.setPower(0);
        shooterMotorRight.setPower(0);

//        shooterServoYaw.setPower(0);
//        shooterServoPitch.setPosition(0);
    }

    // Methods to control shooter
    public void setMotorVelocity(double velocity) {
        shooterMotorLeft.setVelocity(velocity);
        shooterMotorRight.setVelocity(velocity);
    }

    public double getLeftVelocity() {
        return shooterMotorLeft.getVelocity();
    }

    public double getRightVelocity() {
        return shooterMotorRight.getVelocity();
    }

    public void stop() {
        shooterMotorLeft.setPower(0);
        shooterMotorRight.setPower(0);
    }

    public void setTurretYawPower(double p) {
//        shooterServoYaw.setPower(p);
    }

//    public void setServoPosition1(double pos1) {
//        shooterServoYaw.setPosition(pos1);
//    }
//
//    public void setServoPosition2(double pos2) {
//        shooterServoYaw.setPosition(pos2);
//    }

    public void stopShooterMotor() {
        shooterMotorLeft.setPower(0);
        shooterMotorRight.setPower(0);
    }

    public void pointAtGoal() {

    }

    public void shootArtifact() {
    }

    public NormalizedRGBA getColor() {
        return exitSensor.getNormalizedColors();
    }

    public double getExitDistance(DistanceUnit units) {
        return exitSensor.getDistance(units);
    }

    public double getTX() {
        return llResults.getFiducialResults().get(0).getTargetXDegrees();
    }

    public LLResult getLLResults() {
        return llResults;
    }

    public LLStatus getLLStatus() {
        return llStatus;
    }
}
