package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Config
@TeleOp(name = "abg teleop", group = "Competition")
public class Teleop extends OpMode {

    // Dashboard / telemetry
    private FtcDashboard dash;
    private SoftElectronics softElectronics;
    private static Telemetry myTelem;
    private static TelemetryManager myPanels;

    // Mechanisms
    private Drivebase drivebase;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;

    // Drive input
    private double drive = 0;
    private double strafe = 0;
    private double turn = 0;

    // Mechanism power
    private double intakePower = 0.0;
    private double transferPower = 0.0;

    // Shooter velocities
    public static int FAR_SHOT_VELOCITY = 1100;
    public static int CLOSE_SHOT_VELOCITY = 3000;

    public static int shooterDesiredVelocity = 0;

    public static double redTargetAngleFar = 67;
    public static double redTargetAngleClose = 80;
    public static double blueTargetAngleFar = -67;
    public static double blueTargetAngleClose = -80;


    public enum STARTING_ORIENTATION {
        GOAL_SIDE,
        PLAYER_SIDE
    }
    public static STARTING_ORIENTATION startingOrientation = STARTING_ORIENTATION.GOAL_SIDE;

    private enum TURRET_STATE {
        ZEROED,
        ZEROING,
        AIMING_NO_TAG,
        AIMING_TO_TAG
    }

    TURRET_STATE turretState = TURRET_STATE.ZEROED;

    @Override
    public void init() {
        softElectronics = new SoftElectronics(hardwareMap, this.telemetry);
        dash = FtcDashboard.getInstance();
        myTelem = new MultipleTelemetry(dash.getTelemetry(), softElectronics.getTelemetry());
        myPanels = softElectronics.getPanelsTelemetry();

        drivebase = new Drivebase(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        shooterDesiredVelocity = 0;

        myTelem.addData("Status", "Initialized");
        myTelem.update();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        if (gamepad1.rightBumperWasPressed()) {
            onBlueAlliance = true;
        } else if (gamepad1.leftBumperWasPressed()) {
            onBlueAlliance = false;
        }

        if (onBlueAlliance) {
            telemetry.addLine("Blue alliance selected. Press gamepad 1 left bumper to switch.");
            drivebase.offsetYaw(-90);
        } else {
            telemetry.addLine("Red alliance selected. Press gamepad 1 right bumper to switch.");
            drivebase.offsetYaw(90);
        }
    }

    @Override
    public void start() {
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
        drive  = -gamepad1.left_stick_y;
        strafe =  gamepad1.left_stick_x;
        turn   =  gamepad1.right_stick_x;

        drivebase.drive(drive, strafe, turn);

        if (gamepad1.dpad_up) {
            drivebase.resetYaw();
        }

        if (gamepad1.triangle) {
//            drivebase.turnToGoal();
        }
    }

    public void updateTurretState() { //Turret 360: -1484 //Turrent 720: -2954
        double turretFieldHeading = Math.toDegrees(drivebase.getLaserHeading()) + (double) shooter.getTurretPos() / 4.11 * (-1);
        switch (turretState) {
            case ZEROED:
                shooter.setTurretTarget(0);
                break;
            case ZEROING:
                break;
            case AIMING_NO_TAG:
                if (onBlueAlliance) {
//                    if (turretFieldHeading)
                } else {

                }
                break;
            case AIMING_TO_TAG:
                break;
            default:
                break;
        }
    }


    private void updateMechanisms() {

        // LT → Far shot (6000)
        if (gamepad1.left_trigger > 0.3) {
            shooterDesiredVelocity = FAR_SHOT_VELOCITY;
            shooter.setMotorVelocity(shooterDesiredVelocity);
            transfer.openTransferGate();
        }

        // RT → Close shot (4500)
        if (gamepad1.right_trigger > 0.3) {
            shooterDesiredVelocity = CLOSE_SHOT_VELOCITY;
            shooter.setMotorVelocity(shooterDesiredVelocity);
            transfer.openTransferGate();
        }

        // Stop shooter ONLY if both triggers released
        if (gamepad1.left_trigger < 0.3 && gamepad1.right_trigger < 0.3) {
            shooter.stop();
            transfer.closeTransferGate();
            shooterDesiredVelocity = 0;
        }

        if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            intakePower   = 1.0;
            transferPower = 1.0;
        } else if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            intakePower   = -1.0;
            transferPower = -1.0;
        } else {
            intakePower   = 0.0;
            transferPower = 0.0;
        }


        intake.setPower(intakePower);
        transfer.setPower(transferPower);
    }

    private void updateTelemetry() {
        double shooterVel = shooter.getVelocity();
        myTelem.addData("Heading:", Math.toDegrees(drivebase.getPosition().h));
        myTelem.addData("Shooter current velocity: ", shooterVel);
        myTelem.addData("Shooter Target Vel:", shooterDesiredVelocity);
        myTelem.addData("Intake Power:", intakePower);
        myTelem.addData("Transfer Power:", transferPower);
        myTelem.addData("Gate position: ", transfer.getTransferPosition());
        myTelem.update();
    }
}