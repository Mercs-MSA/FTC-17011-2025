package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Drivebase;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Spindex;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;

@Config
@TeleOp
public abstract class Teleop extends LinearOpMode {
    private SoftElectronics softElectronics;
    private Drivebase drivebase;
    private Spindex spindex;
    private Intake intake;
    private Shooter shooter;

    private double drive = 0; // forward/back
    private double strafe = 0; // left/right
    private double turn = 0;  // rotation

    private static Telemetry myTelem;

    private double intakePower = 0.0;

    private enum STATES {STATE1, STATE2, STATE_INACTIVE}
    private static STATES rapidFireState = STATES.STATE_INACTIVE;
    private static STATES motifRapidFireState = STATES.STATE_INACTIVE;
    private static STATES singleShotState = STATES.STATE_INACTIVE;

    @Override
    public void runOpMode() {
        // Initialize SoftElectronics
        softElectronics = new SoftElectronics(hardwareMap, this.telemetry);
        myTelem = softElectronics.getTelemetry();

        // Initialize Drivebase
        drivebase = new Drivebase(hardwareMap);
        spindex = new Spindex(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        myTelem.addData("Status", "Initialized");
        myTelem.update();

        updateDrivebase();
        updateShooter();
        updateIntake();
        updateSpindex();

        updateSoftElectronics();

        updateGamepad();
    }

    private void updateDrivebase() {
        // Field-centric driving
        drivebase.drive(drive, strafe, turn);
    }

    private void updateShooter() {
    }

    private void updateIntake() {
        intake.setPower(intakePower);
    }

    private void updateSpindex() {

    }

    private void updateGamepad() {
        // Example: read joystick inputs
        double drive = -gamepad1.left_stick_y; // forward/back
        double strafe = gamepad1.left_stick_x; // left/right
        double turn = gamepad1.right_stick_x;  // rotation

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
            MotifRapidFire();
        }

        if (gamepad1.right_trigger > 0.5) {
            RapidFire();
        }

        if (gamepad1.right_bumper) {
            SingleShot();
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

    private void RapidFire() {
        runRapidFireStateMachine();
    }

    private void SingleShot() {
        runSingleShotStateMachine();
    }

    private void updateSoftElectronics() {
        myTelem.update();
    }


    /*
     * State machine code
     */

    private void runRapidFireStateMachine() {
        while (opModeIsActive()) {
            switch (rapidFireState) {
                case STATE1:
                    rapidFireState = STATES.STATE2;
                    break;

                case STATE2:
                    rapidFireState = STATES.STATE_INACTIVE;
                    break;

                case STATE_INACTIVE:
                    break;
            }
        }
    }
    private void runMotifRapidFireStateMachine() {
        while (opModeIsActive()) {
            switch (motifRapidFireState) {
                case STATE1:
                    motifRapidFireState = STATES.STATE2;
                    break;

                case STATE2:
                    motifRapidFireState = STATES.STATE_INACTIVE;
                    break;

                case STATE_INACTIVE:
                    break;
            }
        }
    }

    private void runSingleShotStateMachine() {
        while (opModeIsActive()) {
            switch (singleShotState) {
                case STATE1:
                    singleShotState = STATES.STATE2;
                    break;

                case STATE2:
                    singleShotState = STATES.STATE_INACTIVE;
                    break;

                case STATE_INACTIVE:
                    break;
            }
        }
    }
}
