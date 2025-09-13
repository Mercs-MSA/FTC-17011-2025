package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Drivebase;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Spindex;

@Config
@TeleOp
public class Teleop extends OpMode {
    private SoftElectronics softElectronics;
    private Drivebase drivebase;
    private Spindex spindex;
    private Intake intake;
    private Shooter shooter;

    private double drive = -gamepad1.left_stick_y; // forward/back
    private double strafe = gamepad1.left_stick_x; // left/right
    private double turn = gamepad1.right_stick_x;  // rotation

    private static Telemetry myTelem;

    @Override
    public void init() {
        // Initialize SoftElectronics
        softElectronics = new SoftElectronics(this.telemetry);
        myTelem = softElectronics.getTelemetry();

        // Initialize Drivebase
        drivebase = new Drivebase(hardwareMap);
        spindex = new Spindex(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        myTelem.addData("Status", "Initialized");
        myTelem.update();
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

        if (gamepad2.right_bumper) {
            intakePower = 1.0;
        } else if (gamepad2.left_bumper) {
            intakePower = -1.0;
        } else {
            intakePower = 0.0;
        }
    }

    private void updateSoftElectronics() {
        myTelem.update();
    }


    @Override
    public void loop() {
        updateDrivebase();
        updateShooter();
        updateIntake();
        updateSpindex();

        updateSoftElectronics();
    }
}
