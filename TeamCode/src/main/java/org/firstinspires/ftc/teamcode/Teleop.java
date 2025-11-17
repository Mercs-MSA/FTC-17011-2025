package org.firstinspires.ftc.teamcode;

/*
    Control Hub:
    Motors
    0 - frontLeft
    1 - shooterMotorLeft
    2 - (unused / old spindex)
    3 - backLeft

    Expansion Hub:
    Motors
    0 - frontRight
    1 - backRight
    2 - intakeMotor       // single belt intake + transfer
    3 - shooterMotorRight

    I2C
    0 - otos

    External Ethernet - limelight
 */

import static org.firstinspires.ftc.teamcode.Constants.Constants.onBlueAlliance;
import static org.firstinspires.ftc.teamcode.Constants.Constants.ranAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Drivebase;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Config
@TeleOp(name = "Main TeleOp", group = "Competition")
public class Teleop extends OpMode {

    // Dashboard / telemetry
    private FtcDashboard dash;
    private SoftElectronics softElectronics;
    private static Telemetry myTelem;
    private static TelemetryManager myPanels;

    // Mechanisms
    private Drivebase drivebase;
    private Intake intake;
    private Shooter shooter;

    // Drive inputs
    private double drive = 0;   // forward/back
    private double strafe = 0;  // left/right
    private double turn = 0;    // rotation

    // Intake control
    private double intakePower = 0.0;

    // Shooter velocities (ticks/sec) – tune these for your flywheel
    public static int farZoneVelocity   = 6000;  // example: long shot
    public static int closeZoneVelocity = 4500;  // example: close shot
    public static int shooterDesiredVelocity = 6000;

    public enum STARTING_ORIENTATION {
        GOAL_SIDE,
        PLAYER_SIDE
    }

    public static STARTING_ORIENTATION startingOrientation = STARTING_ORIENTATION.GOAL_SIDE;

    @Override
    public void init() {
        // Initialize helper electronics
        softElectronics = new SoftElectronics(hardwareMap, this.telemetry);
        dash = FtcDashboard.getInstance();
        myTelem = new MultipleTelemetry(dash.getTelemetry(), softElectronics.getTelemetry());
        myPanels = softElectronics.getPanelsTelemetry();

        // Initialize mechanisms
        drivebase = new Drivebase(hardwareMap);
        intake    = new Intake(hardwareMap);
        shooter   = new Shooter(hardwareMap);

        shooterDesiredVelocity = farZoneVelocity;

        myTelem.addData("Status", "Initialized");
        myTelem.update();
    }

    @Override
    public void init_loop() {
        // Alliance selection before start
        if (gamepad1.right_bumper) {
            onBlueAlliance = false;
        } else if (gamepad1.left_bumper) {
            onBlueAlliance = true;
        }

        // Set starting yaw based on alliance/orientation
        if (onBlueAlliance) {
            myTelem.addLine("Blue alliance selected. Press right bumper to select red.");
            if (startingOrientation.equals(STARTING_ORIENTATION.GOAL_SIDE)) {
                drivebase.setPosition(new SparkFunOTOS.Pose2D(0, 0, -Math.PI / 2));
            } else {
                drivebase.setPosition(new SparkFunOTOS.Pose2D(0, 0,  Math.PI / 2));
            }
        } else {
            myTelem.addLine("Red alliance selected. Press left bumper to select blue.");
            if (startingOrientation.equals(STARTING_ORIENTATION.GOAL_SIDE)) {
                drivebase.setPosition(new SparkFunOTOS.Pose2D(0, 0,  Math.PI / 2));
            } else {
                drivebase.setPosition(new SparkFunOTOS.Pose2D(0, 0, -Math.PI / 2));
            }
        }

        // Flip orientation with dpad up (edge-detected version)
        if (gamepad1.dpadUpWasPressed()) {
            if (startingOrientation.equals(STARTING_ORIENTATION.PLAYER_SIDE)) {
                startingOrientation = STARTING_ORIENTATION.GOAL_SIDE;
            } else {
                startingOrientation = STARTING_ORIENTATION.PLAYER_SIDE;
            }
        }

        myTelem.update();
    }

    @Override
    public void start() {
        super.start();
        shooter.setMotorVelocity(0);
        ranAuto = false;
    }

    @Override
    public void loop() {
        updateDrivebase();
        updateMechanisms();
        updateTelemetry();
        drivebase.updateLL();
    }

    private void updateDrivebase() {
        // Field-centric driving
        drive  = -gamepad1.left_stick_y;  // forward/back
        strafe =  gamepad1.left_stick_x;  // left/right
        turn   =  gamepad1.right_stick_x; // rotation

        drivebase.drive(drive, strafe, turn);

        // Hold dpad up to hard-reset yaw to 0
        if (gamepad1.dpad_up) {
            drivebase.resetYaw();
        }

        // Auto turn to goal using limelight/heading
        if (gamepad1.triangle) {
            drivebase.turnToGoal();
        }
    }

    private void updateMechanisms() {
        // --- Shooter distance selection ---
        if (gamepad1.left_trigger > 0.5) {
            shooterDesiredVelocity = closeZoneVelocity;
        } else {
            shooterDesiredVelocity = farZoneVelocity;
        }

        // --- Shooter spin ---
        if (gamepad1.right_trigger > 0.5) {
            shooter.setMotorVelocity(shooterDesiredVelocity);
        } else {
            shooter.stop();
        }

        // --- Intake: full power, single motor ---
        // Right bumper: intake IN (positive)
        // Left bumper:  outtake (negative)
        if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            intakePower = 1.0;         // intake into robot
        } else if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            intakePower = -1.0;        // spit out
        } else {
            intakePower = 0.0;
        }

        intake.setPower(intakePower);

        // Edge-triggered yaw reset if you want a clean "snap" button
        if (gamepad1.dpadUpWasPressed()) {
            drivebase.resetYaw();
        }
    }

    private void updateTelemetry() {
        myTelem.addData("Robot Heading (deg):", Math.toDegrees(drivebase.getPosition().h));
        myTelem.addData("Robot Offset (rad):", drivebase.getOffset());
        myTelem.addData("Shooter target vel:", shooterDesiredVelocity);
        myTelem.addData("Shooter right vel:", shooter.getRightVelocity());
        myTelem.addData("Intake power:", intakePower);
        myTelem.update();
    }
}