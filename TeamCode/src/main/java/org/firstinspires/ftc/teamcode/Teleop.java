package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Drivebase;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Spindex;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;

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

    public static int shooterDesiredVelocity = 2000;

    private enum SHOOTER_STATE {REMOVE_USER_CONTROL, INTERMEDIATE_STATE, POINT_AT_GOAL_STATE, RUN_SHOOTER_MOTOR_STATE, CLOSE_GATE_STATE, RUN_TRANSFER_STATE, RUN_SPINDEX_STATE, RELEASE_STATE, INACTIVE_STATE}
    private static SHOOTER_STATE nextState = SHOOTER_STATE.INACTIVE_STATE;
    private static SHOOTER_STATE rapidFireState = SHOOTER_STATE.INACTIVE_STATE;
    private static SHOOTER_STATE motifRapidFireState = SHOOTER_STATE.INACTIVE_STATE;
    private static SHOOTER_STATE singleShotState = SHOOTER_STATE.INACTIVE_STATE;

    private static boolean canDrive = true;

    private static final GeneralConstants.artifactColors[] motifPattern = {GeneralConstants.artifactColors.PURPLE, GeneralConstants.artifactColors.GREEN, GeneralConstants.artifactColors.GREEN};

    private static int motifRapidFireArtifactCycleCount = 0;


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

        myTelem.addData("Status", "Initialized");
        myTelem.update();
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

        myTelem.addData("Right Color:", spindex.getColor(spindex.spindexColorRight) + " " + spindex.getColor(spindex.spindexColorRight, true));
        myTelem.addData("Left Color:", spindex.getColor(spindex.spindexColorLeft) + " " + spindex.getColor(spindex.spindexColorLeft, true));
        myTelem.addData("Back Color:", spindex.getColor(spindex.spindexColorBack) + " " + spindex.getColor(spindex.spindexColorBack, true));
        myTelem.addData("Exit Sensor:", shooter.getDistance(DistanceUnit.CM));
        myTelem.addData("state:", singleShotState.toString());
        myTelem.addData("shooter velocity:", shooter.getRightVelocity());
        myTelem.addData("spindex velocity:", spindex.getSpindexVelocity());
        myTelem.addData("spindex position:", spindex.getSpindexPosition());
        myTelem.update();
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
        updateSingleShotStateMachine();
//        updateMotifRapidFireStateMachine();
//        updateRapidFireStateMachine();
        // Example: read joystick inputs
        if (gamepad1.left_bumper) {
            intakePower = 1.0;
        } else if (gamepad1.dpad_down) {
            intakePower = -1.0;
        } else {
            intakePower = 0;
        }

        intake.setPower(intakePower);

        if (gamepad1.triangle) {
            spindex.runSpindexToColor(GeneralConstants.artifactColors.PURPLE);
        }

        if (gamepad1.circle) {
            spindex.runSpindexToColor(GeneralConstants.artifactColors.GREEN);
        }

        if (gamepad1.left_trigger > 0.5)
            spindex.runSpindex();
//        else if (gamepad1.left_trigger < .5 && gamepad1.left_trigger > .3)
//            spindex.stopSpindex();

        if (gamepad1.right_trigger > 0.5) {
            spindex.openSpindexGate();
//            spindex.runTransferWheel();
        }

        if (gamepad1.right_bumper) {
//            singleShotState = SHOOTER_STATE.RUN_SPINDEX_STATE;
//            singleShotState = SHOOTER_STATE.RUN_SHOOTER_MOTOR_STATE;
            shooter.setMotorVelocity(shooterDesiredVelocity);
//            if (shooter.getRightVelocity() > 2000)
//                spindex.runTransferWheel();
        }

        if (gamepad1.cross) {
            shooter.stop();
            spindex.stopTransferWheel();
            spindex.stopSpindex();
            spindex.closeSpindexGate();
        }

        if (gamepad1.dpad_up) {
            singleShotState = SHOOTER_STATE.REMOVE_USER_CONTROL;
        }
    }

//    private void MotifRapidFire() {
//        GeneralConstants.artifactColors[] motifPattern = {GeneralConstants.artifactColors.PURPLE, GeneralConstants.artifactColors.GREEN, GeneralConstants.artifactColors.GREEN};
//
//        spindex.runSpindexToColor(motifPattern[0]);
//        shooter.shootArtifact();
//
//        spindex.runSpindexToColor(motifPattern[1]);
//        shooter.shootArtifact();
//
//        spindex.runSpindexToColor(motifPattern[2]);
//        shooter.shootArtifact();
//    }



    /*
     * State machine code
     */

    private void updateRapidFireStateMachine() {
        switch (rapidFireState) {

            case REMOVE_USER_CONTROL:
                canDrive = false;

                rapidFireState = SHOOTER_STATE.POINT_AT_GOAL_STATE;
                break;

            case POINT_AT_GOAL_STATE:
//                shooter.pointAtGoal();


                if (Math.abs(shooter.getCurrentAngle() - shooter.getGoalAngle()) <= 0.2) {
                    rapidFireState = SHOOTER_STATE.RUN_SHOOTER_MOTOR_STATE;
                }
                break;

            case RUN_SHOOTER_MOTOR_STATE:
                shooter.setMotorVelocity(shooterDesiredVelocity);

                if (Math.abs(shooter.getLeftVelocity() - shooterDesiredVelocity) <= 0.2) {
                    if (Math.abs(shooter.getRightVelocity()  - shooterDesiredVelocity) <= 0.2) {
                        rapidFireState = SHOOTER_STATE.CLOSE_GATE_STATE;
                    }
                }

                break;

            case CLOSE_GATE_STATE:
                spindex.closeSpindexGate();

                rapidFireState = SHOOTER_STATE.RUN_TRANSFER_STATE;
                break;

            case RUN_TRANSFER_STATE:
                spindex.runTransferWheel();

                rapidFireState = SHOOTER_STATE.RUN_SPINDEX_STATE;

                rapidFireTimer.reset();
                break;

            case RUN_SPINDEX_STATE:

                spindex.runSpindex();

                if (rapidFireTimer.time() > 3)
                    rapidFireState = SHOOTER_STATE.INACTIVE_STATE;
                break;

            case INACTIVE_STATE:
                canDrive = true;
                shooter.stopShooterMotor();
                spindex.stopSpindex();
                spindex.stopTransferWheel();
                break;
        }
    }
    private void updateMotifRapidFireStateMachine() {
        switch (motifRapidFireState) {

            case REMOVE_USER_CONTROL:
                canDrive = false;
                motifRapidFireArtifactCycleCount = 0;

                motifRapidFireState = SHOOTER_STATE.POINT_AT_GOAL_STATE;
                break;

            case POINT_AT_GOAL_STATE:
                shooter.pointAtGoal();

                if (Math.abs(shooter.getCurrentAngle() - shooter.getGoalAngle()) <= 0.2) {
                    motifRapidFireState = SHOOTER_STATE.RUN_SPINDEX_STATE;
                }
                break;

            case RUN_SPINDEX_STATE:
                spindex.runSpindex();
                spindex.stopTransferWheel();

                if (spindex.getColor(spindex.spindexColorBack) == motifPattern[motifRapidFireArtifactCycleCount]) {
                    motifRapidFireState = SHOOTER_STATE.RUN_TRANSFER_STATE;
                    rapidFireTimer.reset();
                }
                break;

            case RUN_TRANSFER_STATE:
                spindex.stopSpindex();
                spindex.runTransferWheel();

                if (rapidFireTimer.time() > 3) {
                    motifRapidFireArtifactCycleCount++;

                    if (motifRapidFireArtifactCycleCount >= motifPattern.length)
                        motifRapidFireState = SHOOTER_STATE.INACTIVE_STATE;
                    else
                        motifRapidFireState = SHOOTER_STATE.RUN_SPINDEX_STATE;
                }
                break;

            case INACTIVE_STATE:
                canDrive = true;
                shooter.stopShooterMotor();
                spindex.stopSpindex();
                spindex.stopTransferWheel();

                motifRapidFireArtifactCycleCount = 0;
                break;
        }
    }

    private void updateSingleShotStateMachine() {
        switch (singleShotState) {
            case INTERMEDIATE_STATE: whileRunning(); break;
//            case RUN_SPINDEX_STATE:
//                spindex.runSpindexToColor(GeneralConstants.artifactColors.PURPLE);
//                singleShotState = SHOOTER_STATE.RUN_SHOOTER_MOTOR_STATE;
//                break;
            case REMOVE_USER_CONTROL:
                canDrive = false;
                singleShotState = SHOOTER_STATE.RUN_SHOOTER_MOTOR_STATE;
                break;
            case RUN_SHOOTER_MOTOR_STATE:
                shooter.setMotorVelocity(shooterDesiredVelocity);
                if (shooter.getRightVelocity() > shooterDesiredVelocity * .99) {
                    singleShotState = SHOOTER_STATE.RUN_TRANSFER_STATE;
                }
                break;
            case RUN_SPINDEX_STATE:
//                if (!spindex.getColor(spindex.spindexColorBack).equals(GeneralConstants.artifactColors.EMPTY))
                    singleShotState = SHOOTER_STATE.RUN_TRANSFER_STATE;
//                runTransfer();
//                spindex.closeSpindexGate();
                break;
            case RUN_TRANSFER_STATE:
                spindex.runTransferWheel();
                if (shooter.getDistance(DistanceUnit.CM) < 4) {
                    myTelem.addData("Shot", "");
                    singleShotState = SHOOTER_STATE.INACTIVE_STATE;
                }
                break;
            case INACTIVE_STATE:
                holdPower();
                break;
        }
    }

    public void whileRunning() {
    }

    public void runTransfer() {
        singleShotState = SHOOTER_STATE.INTERMEDIATE_STATE;
        spindex.runTransferWheel();
    }

    private void holdPower() {
        shooter.stop();
    }
}

