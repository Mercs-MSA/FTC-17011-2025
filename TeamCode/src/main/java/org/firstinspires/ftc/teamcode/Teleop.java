package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Drivebase;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Spindex;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;

@Config
@TeleOp
public class Teleop extends OpMode {
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

    private double intakePower = 0.0;

    public static int shooterDesiredVelocity = 60;

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
        myTelem = softElectronics.getTelemetry();

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
    public void loop() {
        updateDrivebase();
        updateShooter();
        updateIntake();
        updateSpindex();

        updateSoftElectronics();

        updateGamepad();
        myTelem.addData("state:", singleShotState.toString());
        myTelem.update();
    }

    private void updateDrivebase() {
        // Field-centric driving
        if (singleShotState != SHOOTER_STATE.INACTIVE_STATE || rapidFireState != SHOOTER_STATE.INACTIVE_STATE || motifRapidFireState != SHOOTER_STATE.INACTIVE_STATE) {
            drivebase.stop();
        } else {
            drivebase.drive(drive, strafe, turn);
        }

        if (gamepad2.right_bumper) {
            drivebase.setFrontRightPower(.3);
            drivebase.setBackRightPower(.3);
            drivebase.setFrontLeftPower(.3);
            drivebase.setBackLeftPower(.3);
        }
    }

    private void updateShooter() {
        updateSingleShotStateMachine();
        updateMotifRapidFireStateMachine();
        updateRapidFireStateMachine();
    }

    private void updateIntake() {
        intake.setPower(intakePower);
    }

    private void updateSpindex() {
    }

    private void updateGamepad() {
        // Example: read joystick inputs
        drive = -gamepad1.left_stick_y; // forward/back
        strafe = gamepad1.left_stick_x; // left/right
        turn = gamepad1.right_stick_x;  // rotation

        if (gamepad1.left_bumper) {
            intakePower = 1.0;
        } else if (gamepad1.dpad_down) {
            intakePower = -1.0;
        }

        if (gamepad1.a) {
            spindex.runSpindexToColor(GeneralConstants.artifactColors.PURPLE);
        }

        if (gamepad1.b) {
            spindex.runSpindexToColor(GeneralConstants.artifactColors.GREEN);
        }

        if (gamepad1.left_trigger > 0.5) {
//            MotifRapidFire();
            spindex.stopSpindex();
        }

        if (gamepad1.right_trigger > 0.5) {
            spindex.runSpindex();
        }

        if (gamepad1.right_bumper) {
//            singleShotState = SHOOTER_STATE.RUN_SPINDEX_STATE;
            singleShotState = SHOOTER_STATE.RUN_SHOOTER_MOTOR_STATE;
        }


    }

    private void MotifRapidFire() {
        GeneralConstants.artifactColors[] motifPattern = {GeneralConstants.artifactColors.PURPLE, GeneralConstants.artifactColors.GREEN, GeneralConstants.artifactColors.GREEN};

        spindex.runSpindexToColor(motifPattern[0]);
        shooter.shootArtifact();

        spindex.runSpindexToColor(motifPattern[1]);
        shooter.shootArtifact();

        spindex.runSpindexToColor(motifPattern[2]);
        shooter.shootArtifact();
    }


    private void updateSoftElectronics() {
        myTelem.update();
    }


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
            case RUN_SHOOTER_MOTOR_STATE:
                shooter.setMotorVelocity(60);
                if (shooter.getRightVelocity() > 55) {
                    singleShotState = SHOOTER_STATE.RUN_TRANSFER_STATE;
                }
                break;
            case RUN_TRANSFER_STATE: runTransfer();
//                spindex.closeSpindexGate();
                break;
            case INACTIVE_STATE:
                holdPower();
                break;
        }
    }

    public void whileRunning() {
        myTelem.addData("i:", i);
        if (i == 499) singleShotState = SHOOTER_STATE.INACTIVE_STATE;
    }

    int i = 0;

    public void runTransfer() {
        singleShotState = SHOOTER_STATE.INTERMEDIATE_STATE;
        spindex.runTransferWheel();
        while (i < 500) {
            i++;
        }
    }

    private void holdPower() {
        shooter.stop();
    }
}

