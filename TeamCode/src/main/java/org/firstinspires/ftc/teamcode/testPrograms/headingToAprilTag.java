package org.firstinspires.ftc.teamcode.testPrograms;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.SoftElectronics;
import org.firstinspires.ftc.teamcode.mechanisms.Drivebase;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Spindex;

import java.util.List;

@Config
@TeleOp
public class headingToAprilTag extends OpMode {
    private SoftElectronics softElectronics;
//    private Drivebase drivebase;
    private Spindex spindex;
    private Intake intake;
    private Shooter shooter;

    private Limelight3A limelight;

    private DcMotor frontLeft, frontRight, backLeft, backRight;


//    private double drive = -gamepad1.left_stick_y; // forward/back
//    private double strafe = gamepad1.left_stick_x; // left/right
//    private double turn = gamepad1.right_stick_x;  // rotation

    private static Telemetry myTelem;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        limelight.start();

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Initialize SoftElectronics
        softElectronics = new SoftElectronics(this.telemetry);
        myTelem = softElectronics.getTelemetry();
        myTelem.setMsTransmissionInterval(11);


        // Initialize Drivebase
//        drivebase = new Drivebase(hardwareMap);
//        spindex = new Spindex(hardwareMap);
//        intake = new Intake(hardwareMap);
//        shooter = new Shooter(hardwareMap);

        myTelem.addData("Status", "Initialized");
        myTelem.update();
    }

    private void updateDrivebase() {
        // Field-centric driving
//        drivebase.drive(drive, strafe, turn);

        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

        // Send calculated power to wheels
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    private void updateShooter() {

    }

    private void updateIntake() {
        intake.setPower(intakePower);
    }

    private void updateSpindex() {

    }

    private void updateLL() {
        LLStatus status = limelight.getStatus();
        myTelem.addData("Name", "%s",
                status.getName());
        myTelem.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        myTelem.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access general information
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            myTelem.addData("LL Latency", captureLatency + targetingLatency);
            myTelem.addData("Parse Latency", parseLatency);
            myTelem.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            double targetX = result.getFiducialResults().get(0).getTargetXDegrees();

            myTelem.addData("targetX", targetX);

            myTelem.addData("Botpose", botpose.toString());

            // Access barcode results
//            List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
//            for (LLResultTypes.BarcodeResult br : barcodeResults) {
//                myTelem.addData("Barcode", "Data: %s", br.getData());
//            }

//            // Access classifier results
//            List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
//            for (LLResultTypes.ClassifierResult cr : classifierResults) {
//                myTelem.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
//            }

            // Access detector results
//            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
//            for (LLResultTypes.DetectorResult dr : detectorResults) {
//                myTelem.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
//            }

            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                myTelem.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }

            // Access color results
//            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//            for (LLResultTypes.ColorResult cr : colorResults) {
//                myTelem.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
//            }

            boolean adjust = targetX > 2 || targetX < -2;
            myTelem.addData("Needs adjust?", adjust);

            if (gamepad1.cross && adjust) {
                myTelem.addData("ADJUSTING!!", true);
                if (targetX > 2) {
                    frontRight.setPower(-0.3);
                    backRight.setPower(-0.3);
                    frontLeft.setPower(0.3);
                    backLeft.setPower(0.3);
                } else {
                    frontRight.setPower(0.3);
                    backRight.setPower(0.3);
                    frontLeft.setPower(-0.3);
                    backLeft.setPower(-0.3);

                }
            } else {
                myTelem.addData("ADJUSTING!!", false);
                frontRight.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                backLeft.setPower(0);
            }
        } else {
            myTelem.addData("Limelight", "No data available");
            if (gamepad1.cross) {
                frontRight.setPower(0.3);
                backRight.setPower(0.3);
                frontLeft.setPower(-0.3);
                backLeft.setPower(-0.3);
            }
        }




    }

    private void updateGamepad() {
        // Example: read joystick inputs
//        double drive = -gamepad1.left_stick_y; // forward/back
//        double strafe = gamepad1.left_stick_x; // left/right
//        double turn = gamepad1.right_stick_x;  // rotation
//
//        if (gamepad2.right_bumper) {
//            intakePower = 1.0;
//        } else if (gamepad2.left_bumper) {
//            intakePower = -1.0;
//        } else {
//            intakePower = 0.0;
//        }
    }

    private void updateSoftElectronics() {
        myTelem.update();
    }

    private void updateEverything() {
//        updateDrivebase();
//        updateShooter();
//        updateIntake();
//        updateSpindex();
//
//        updateSoftElectronics();
        updateLL();
    }

    @Override
    public void loop() {
        updateEverything();
    }
}
