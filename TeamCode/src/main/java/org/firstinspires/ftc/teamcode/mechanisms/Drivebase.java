package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.Constants.Constants.onBlueAlliance;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SoftElectronics;


@Config
public class Drivebase {

    // Declare motors
    private static DcMotor frontLeft, frontRight, backLeft, backRight;
    public static SparkFunOTOS otos;
    private double offset = 0;
    private double IMUheadingTracker = 0;

    private Limelight3A limelight;
    private LLStatus llStatus;
    private LLResult llResults;
    public static double kP = 0.01;

    private double TX = 0;

    public static double blueAimPointX = 7.5;
    public static double blueAimPointy = 144.0;

    public static double redAimPointX = 160.5;
    public static double redAimPointy = 144.0;

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
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);
        otos.setAngularScalar(.985889);
        otos.resetTracking();
        otos.calibrateImu();

        TX = 0;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(onBlueAlliance ? 0 : 1);
    }


    public void offsetYaw(double offset) {
        this.offset = Math.toRadians(offset);
//        otos.setOffset(new SparkFunOTOS.Pose2D(0,0,Math.toRadians(offset)));
    }

    public void resetYaw() {
            otos.setPosition(new SparkFunOTOS.Pose2D(0,0,0));
    }

    public void updateLL() {
        llStatus = limelight.getStatus();
        llResults = limelight.getLatestResult();

        if (llResults.isValid()) {
            double captureLatency = llResults.getCaptureLatency();
            double targetingLatency = llResults.getTargetingLatency();
            double parseLatency = llResults.getParseLatency();
        }
    }

    public static double getPointsHeading(double x, double y, double xr, double yr)
    {
        double calculatedAngleRads = Math.atan2(y-yr, x-xr);
        double calculatedAngleDegs = Math.toDegrees(calculatedAngleRads);
        //double correctedAngle = calculatedAngleDegs - 90.0;
        return calculatedAngleDegs;
    }


    // Field-centric drive
    public void drive(double drive, double strafe, double turn) {
        // Get current heading
//        double botHeading = SoftElectronics.getYaw() + offset;
        double botHeading = otos.getPosition().h + offset;
        IMUheadingTracker = botHeading;

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
        otos.setPosition(pose);
    }
    public static double angleWrap(double angle) {
//        angle = angle % 360;
//
//        if (angle > 180)
//            angle -= 360;
//        if (angle <= -180)
//            angle += 360;

        return angle;
    }

    public void turnToHeading(double targetHeading) {
        // convert bot heading to [-180, 180]
        double currentHeading = AngleUnit.DEGREES.normalize(Math.toDegrees(otos.getPosition().h));

        // smallest rotation
        double error = AngleUnit.DEGREES.normalize((targetHeading - currentHeading) * -1);

        double turnPower = error * kP;

//        if (turnPower > 0) {
//            turnPower = Math.min(turnPower, 0.267);
//        } else if (turnPower < 0) {
//            turnPower = Math.max(turnPower, -0.267);
//        }

        turnPower = Range.clip(turnPower, -0.267, 0.267);

        setDrivePower(turnPower, -turnPower, turnPower, -turnPower);
    }

    public void setDrivePower(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
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
//    public boolean getTXInRange() {return llResults.getFiducialResults().get(0).getTargetXDegrees() <= .5 && llResults.getFiducialResults().get(0).getTargetXDegrees() >= -.5;}
    public boolean getTargetSeen() {return llResults.isValid();}
    public double getTX() {return llResults.getFiducialResults().get(0).getTargetXDegrees();}
    public LLResult getResults() {return llResults;}
    public double getOffset() {
        return offset;
    }
}
