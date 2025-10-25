package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Drivebase;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Spindex;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class Teleop extends OpMode {
    public FtcDashboard dash;
    private SoftElectronics softElectronics;
    private Drivebase drivebase;
    private Spindex spindex;
    private Intake intake;
    private Shooter shooter;

    private double drive = 0; // forward/back
    private double strafe = 0; // left/right
    private double turn = 0;  // rotation

    private ElapsedTime rapidFireTimer;

    private static Telemetry myTelem;
    private static TelemetryManager myPanels;

    private double intakePower = 0.0;

    public static int shooterDesiredVelocity = 4000;

    public static boolean onBlueAlliance = true;

    public enum STARTING_ORIENTATION {
        GOAL_SIDE,
        PLAYER_SIDE
    }

    public static STARTING_ORIENTATION startingOrientation = STARTING_ORIENTATION.GOAL_SIDE;

    private enum SHOOTER_STATE {START_STATE, POINT_AT_GOAL_STATE, RUN_SHOOTER_MOTOR_STATE, CLOSE_GATE_STATE, RUN_TRANSFER_STATE, RUN_SPINDEX_STATE, RELEASE_STATE, INACTIVE_STATE, END_STATE}
    private static SHOOTER_STATE rapidFireState = SHOOTER_STATE.INACTIVE_STATE;
    private static SHOOTER_STATE motifRapidFireState = SHOOTER_STATE.INACTIVE_STATE;
    private static SHOOTER_STATE singleShotState = SHOOTER_STATE.INACTIVE_STATE;
    public static boolean spinningToColor = false;

    ElapsedTime shotTimer = new ElapsedTime();
    double ema = 0;                             // exponential moving average of RPM
    final double ALPHA = 0.2;                   // smoothing factor (0..1)
    final double DROP_PCT = 0.18;               // 18% dip counts as “ball contact”
    final double RECOVER_PCT = 0.10;            // must recover within 10% of target to re-arm
    boolean shotArmed = true;
    boolean shotDetected = false;

    @Override
    public void init() {
        // Initialize SoftElectronics
        softElectronics = new SoftElectronics(hardwareMap, this.telemetry);
        dash = FtcDashboard.getInstance();
        myTelem = new MultipleTelemetry(dash.getTelemetry(), softElectronics.getTelemetry());
        myPanels = softElectronics.getPanelsTelemetry();

        // Initialize Drive base
        drivebase = new Drivebase(hardwareMap);
        spindex = new Spindex(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        rapidFireTimer = new ElapsedTime();

        singleShotState = SHOOTER_STATE.INACTIVE_STATE;
        rapidFireState = SHOOTER_STATE.INACTIVE_STATE;
        myTelem.addData("Status", "Initialized");
        myTelem.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.right_bumper)
            onBlueAlliance = false;
        else if (gamepad1.left_bumper)
            onBlueAlliance = true;

        if (onBlueAlliance) {
            myTelem.addLine("Blue alliance selected. Press right bumper to select red.");
            if (startingOrientation.equals(STARTING_ORIENTATION.GOAL_SIDE)) {
//                drivebase.offsetYaw(-90);
                drivebase.setPosition(new SparkFunOTOS.Pose2D(0, 0, -Math.PI/2));
            } else {
//                drivebase.offsetYaw(90);
                drivebase.setPosition(new SparkFunOTOS.Pose2D(0, 0, Math.PI/2));
            }
        } else {
            myTelem.addLine("Red alliance selected. Press left bumper to select blue.");
            if (startingOrientation.equals(STARTING_ORIENTATION.GOAL_SIDE)) {
//                drivebase.offsetYaw(90);
                drivebase.setPosition(new SparkFunOTOS.Pose2D(0, 0, Math.PI/2));
            } else {
//                drivebase.offsetYaw(-90);
                drivebase.setPosition(new SparkFunOTOS.Pose2D(0, 0, -Math.PI/2));
            }
        }

//        if (startingOrientation.equals(STARTING_ORIENTATION.PLAYER_SIDE)) {
//            myTelem.addLine("Facing towards player side. Press dpad up to select goal side.");
//            drivebase.setPosition(new SparkFunOTOS.Pose2D(0, 0, Math.PI));
//        } else if (startingOrientation.equals(STARTING_ORIENTATION.GOAL_SIDE)) {
//            myTelem.addLine("Facing towards goal side. Press dpad up to select player side.");
//            drivebase.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
//        }

        if (gamepad1.dpadUpWasPressed()) {
            if (startingOrientation.equals(STARTING_ORIENTATION.PLAYER_SIDE))
                startingOrientation = STARTING_ORIENTATION.GOAL_SIDE;
            else
                startingOrientation = STARTING_ORIENTATION.PLAYER_SIDE;
        }
    }

    @Override
    public void start() {
        super.start();
        shooter.setShooterPower(1);
        shooter.setMotorVelocity(0);
        spindex.initSpindex();
    }

    @Override
    public void loop() {
        updateDrivebase();
        updateMechanisms();
        updateTelemetry();
        shooter.updateLL();
    }

    private void updateTelemetry() {
        //        myTelem.addData("Robot Yaw:", Math.toDegrees(drivebase.getPosition().h));
        myTelem.addData("Robot Heading:", drivebase.getBotHeading());
        myTelem.addData("Robot Offset:", drivebase.getOffset());
//        myTelem.addData("Spinning to Color?", spinningToColor);
//        myTelem.addData("Num of artifacts in robot:", spindex.getNumOfArtifactsInRobot());
        myTelem.addData("Right Color:", spindex.getColor(spindex.spindexColorRight));
        myTelem.addData("Left Color:", spindex.getColor(spindex.spindexColorLeft));
        myTelem.addData("Back Color:", spindex.getColor(spindex.spindexColorBack));
        myTelem.addData("single shot state:", singleShotState.toString());
        myTelem.addData("rapid fire state:", rapidFireState.toString());
        myTelem.addData("rapid fire timer:", rapidFireTimer.time(TimeUnit.SECONDS));
        myTelem.addData("shooter velocity:", shooter.getRightVelocity());
        myTelem.addData("spindex velocity:", spindex.getSpindexVelocity());
        myTelem.addData("spindex position:", spindex.getSpindexPosition());
        telemetry.addData("result valid?", shooter.getLLResults().isValid());
        telemetry.addData("pipeline", shooter.getLLStatus().getPipelineIndex());
        telemetry.addData("TX", shooter.getTX() == null ? "null" : shooter.getTX());
        telemetry.addData("inRange", shooter.inRange());
        myTelem.update();
    }

    private void updateShotDetector() {
        double rpm = shooter.getRpm();
        ema = (ALPHA * rpm) + (1 - ALPHA) * ema;

        // arm when we’re basically at speed
        if (!shotArmed && ema > shooterDesiredVelocity * (1.0 - RECOVER_PCT)) {
            shotArmed = true;
        }

        // detect dip
        if (shotArmed && ema < shooterDesiredVelocity * (1.0 - DROP_PCT)) {
            shotDetected = true;
            shotArmed = false;               // prevent double-count
            shotTimer.reset();
        }

        // optional: clear flag after a short window so you can edge-trigger it
        if (shotDetected && shotTimer.seconds() > 0.25) {
            shotDetected = false;
        }
    }

    private void updateDrivebase() {
        // Field-centric driving
        drive = -gamepad1.left_stick_y; // forward/back
        strafe = gamepad1.left_stick_x; // left/right
        turn = gamepad1.right_stick_x;  // rotation

        if (singleShotState != SHOOTER_STATE.INACTIVE_STATE || rapidFireState != SHOOTER_STATE.INACTIVE_STATE || motifRapidFireState != SHOOTER_STATE.INACTIVE_STATE) {
            drivebase.stop();
        } else {
            drivebase.drive(drive, strafe, turn);
        }
    }

    private void updateMechanisms() {

        /* * COMPETITION ROBOT BUTTONS: * */
        updateSingleShotStateMachine();
        updateRapidFireStateMachine();
        updateShotDetector();

        //Intake
        if (gamepad1.left_bumper) {
            intakePower = -0.5;
            spindex.runSpindex();
        } else if (gamepad1.right_bumper) {
            intakePower = 0.5;
        } else if (rapidFireState.equals(SHOOTER_STATE.INACTIVE_STATE)) {
            intakePower = 0;
            spindex.stopSpindex();
        }
        intake.setPower(intakePower);

        //Shooter
        if (gamepad1.left_trigger > 0.5 && rapidFireState.equals(SHOOTER_STATE.INACTIVE_STATE)) {
            rapidFireState = SHOOTER_STATE.START_STATE;
        }

        /* * MOTOR TESTING BUTTONS: * */

        //Transfer
        if (gamepad1.a) {
            spindex.runTransferWheel();
        } else if (rapidFireState.equals(SHOOTER_STATE.INACTIVE_STATE)) {
            spindex.stopTransferWheel();
        }

        //Spindex
        if (gamepad1.b) {
            spindex.runSpindex();
        } else if (rapidFireState.equals(SHOOTER_STATE.INACTIVE_STATE)) {
            spindex.stopSpindex();
        }

        //Shooter On
        if (gamepad1.x) {
            shooter.setMotorVelocity(shooterDesiredVelocity);
        }

        //Shooter Off
        if (gamepad1.y) {
            shooter.setMotorVelocity(0);
        }


    }

    private void updateRapidFireStateMachine() {
        switch (rapidFireState) {
            case START_STATE:
                rapidFireState = SHOOTER_STATE.RUN_SHOOTER_MOTOR_STATE;
                break;

            case RUN_SHOOTER_MOTOR_STATE:
                intake.setPower(0.5);
                shooter.setMotorVelocity(shooterDesiredVelocity);

                //Go to Next State
                rapidFireState = SHOOTER_STATE.RUN_SPINDEX_STATE;

                //Cancel State Machine
                if (drive > .1 || drive < -.1 || strafe > .1 || strafe < -.1 || turn > .1 || turn < -.1) {
                    rapidFireState = SHOOTER_STATE.END_STATE;
                }
                break;

            case RUN_SPINDEX_STATE:
                spindex.runSpindex();
                spindex.stopTransferWheel(); //might need to comment this

                //Go to next state when Artifact is in position AND shooter has reached desired velocity
                if (!spindex.getColor(spindex.spindexColorBack).equals(GeneralConstants.artifactColors.EMPTY) && shooter.getRightVelocity() > shooterDesiredVelocity * .95) {
                    rapidFireState = SHOOTER_STATE.RUN_TRANSFER_STATE;
                    rapidFireTimer.reset();
                }

                //Cancel State Machine
                if (drive > .1 || drive < -.1 || strafe > .1 || strafe < -.1 || turn > .1 || turn < -.1) {
                    rapidFireState = SHOOTER_STATE.END_STATE;
                }
                break;

            case RUN_TRANSFER_STATE:
                spindex.stopSpindex();
                spindex.runTransferWheel();

                //Repeat RUN_SPINDEX State when timer has reached 3 seconds or when artifact is shot
                if (rapidFireTimer.time(TimeUnit.SECONDS) > 3 || shotDetected) {
                    rapidFireTimer.reset();
                    rapidFireState = SHOOTER_STATE.RUN_SPINDEX_STATE;
                }

                //Cancel State Machine
                if (drive > .1 || drive < -.1 || strafe > .1 || strafe < -.1 || turn > .1 || turn < -.1) {
                    rapidFireState = SHOOTER_STATE.END_STATE;
                }

                break;

            case END_STATE:
                shooter.stop();
                spindex.stopTransferWheel();
                spindex.stopSpindex();

                rapidFireState = SHOOTER_STATE.INACTIVE_STATE;

                break;
            case INACTIVE_STATE:
                break;
        }
    }
    private void updateSingleShotStateMachine() {
        switch (singleShotState) {
            case START_STATE:
                singleShotState = SHOOTER_STATE.RUN_SHOOTER_MOTOR_STATE;
                break;
            case RUN_SHOOTER_MOTOR_STATE:
                intake.setPower(1);
                shooter.setMotorVelocity(shooterDesiredVelocity);
                if (shooter.getRightVelocity() > shooterDesiredVelocity * .8) {
                    singleShotState = SHOOTER_STATE.RUN_SPINDEX_STATE;
                }
                break;
            case RUN_SPINDEX_STATE:
                intake.setPower(1);
                spindex.runSpindexToNextArtifact(2);
                spindex.stopTransferWheel();
                if (!spindex.getColor(spindex.spindexColorBack).equals(GeneralConstants.artifactColors.EMPTY) && shooter.getRightVelocity() > shooterDesiredVelocity * .95)
                    singleShotState = SHOOTER_STATE.RUN_TRANSFER_STATE;
                break;
            case RUN_TRANSFER_STATE:
                intake.setPower(1);
                spindex.stopSpindex();
                spindex.runTransferWheel();
                if ((drive > .1 || drive < -.1 || strafe > .1 || strafe < -.1 || turn > .1 || turn < -.1) || shotDetected)
                    singleShotState = SHOOTER_STATE.INACTIVE_STATE;
                break;
            case INACTIVE_STATE:
                if (rapidFireState.equals(SHOOTER_STATE.INACTIVE_STATE)) {
                    shooter.stop();
                    spindex.stopTransferWheel();
                }
                break;
        }
    }
}


