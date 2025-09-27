package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    private static Telemetry myTelem;

    private double intakePower = 0.0;

    private enum STATES {STATE1, STATE2, STATE_INACTIVE}

    private static STATES rapidFireState = STATES.STATE_INACTIVE;
    private static STATES motifRapidFireState = STATES.STATE_INACTIVE;
    private static STATES singleShotState = STATES.STATE_INACTIVE;


    @Override
    public void init() {
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
    }

    @Override
    public void loop() {
        updateDrivebase();
        updateShooter();
        updateIntake();
        updateSpindex();

        updateSoftElectronics();

        updateGamepad();
    }

    private void updateDrivebase() {
        // Field-centric driving
        if (singleShotState != STATES.STATE_INACTIVE || rapidFireState != STATES.STATE_INACTIVE || motifRapidFireState != STATES.STATE_INACTIVE) {
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
        runSingleShotStateMachine();
        runMotifRapidFireStateMachine();
        runRapidFireStateMachine();
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
            MotifRapidFire();
        }

        if (gamepad1.right_trigger > 0.5) {
//            RapidFire();
        }

        if (gamepad1.right_bumper) {
            singleShotState = STATES.STATE1;
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

    private void runRapidFireStateMachine() {
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
    private void runMotifRapidFireStateMachine() {
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

    private void runSingleShotStateMachine() {
        switch (singleShotState) {
            case STATE1:
                spinUp();
                break;

            case STATE2:
                shoot();
                break;

            case STATE_INACTIVE:
                holdPower();
                break;
        }
    }

    private void spinUp() {
        shooter.setMotorVelocity(1600);
        if (shooter.getRightVelocity() == 1600)
            singleShotState = STATES.STATE2;
    }

    private void shoot() {
        spindex.runTransferWheel();
        if (shooter.getRightVelocity() < 1600)
            singleShotState = STATES.STATE_INACTIVE;
    }

    private void holdPower() {
        shooter.stop();
    }
}

