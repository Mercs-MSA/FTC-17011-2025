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

import org.firstinspires.ftc.teamcode.mechanisms.Drivebase;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Config
@TeleOp(name = "Main TeleOp", group = "Competition")
public class Teleop extends OpMode {

    private FtcDashboard dash;
    private SoftElectronics softElectronics;
    private Drivebase drivebase;
    private Intake intake;
    private Shooter shooter;

    private double drive = 0;
    private double strafe = 0;
    private double turn = 0;

    private double intakePower = 0.0;

    // Shooter velocity presets (ticks/sec)
    public static int farZoneVelocity   = 2000;
    public static int closeZoneVelocity = 1250;
    public static int shooterDesiredVelocity = 2000;

    @Override
    public void init() {
        softElectronics = new SoftElectronics(hardwareMap, this.telemetry);
        dash = FtcDashboard.getInstance();

        // Telemetry
        MultipleTelemetry myTelem = new MultipleTelemetry(dash.getTelemetry(), softElectronics.getTelemetry());

        // Mechanisms
        drivebase = new Drivebase(hardwareMap);
        intake    = new Intake(hardwareMap);
        shooter   = new Shooter(hardwareMap);

        shooterDesiredVelocity = farZoneVelocity;

        myTelem.addData("Status", "Initialized");
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
        drive  = -gamepad1.left_stick_y;
        strafe =  gamepad1.left_stick_x;
        turn   =  gamepad1.right_stick_x;

        drivebase.drive(drive, strafe, turn);

        if (gamepad1.dpad_up)   drivebase.resetYaw();
        if (gamepad1.triangle)  drivebase.turnToGoal();
    }

    private void updateMechanisms() {

        // Select shooting velocity
        if (gamepad1.left_trigger > 0.5)
            shooterDesiredVelocity = closeZoneVelocity;
        else
            shooterDesiredVelocity = farZoneVelocity;

        // Spin-up shooter
        if (gamepad1.right_trigger > 0.5)
            shooter.setMotorVelocity(shooterDesiredVelocity);
        else
            shooter.stop();

        // Intake controls
        if (gamepad1.right_bumper)
            intakePower = 1.0;
        else if (gamepad1.left_bumper)
            intakePower = -1.0;
        else
            intakePower = 0.0;

        intake.setPower(intakePower);

        if (gamepad1.dpadUpWasPressed())
            drivebase.resetYaw();
    }

    private void updateTelemetry() {
        telemetry.addData("Heading (deg)", Math.toDegrees(drivebase.getPosition().h));
        telemetry.addData("Shooter Target Vel", shooterDesiredVelocity);
        telemetry.addData("Shooter Vel", shooter.getVelocity());  // SINGLE MOTOR
        // telemetry.addData("Shooter Right Vel", shooter.getRightVelocity()); // OLD
        telemetry.addData("Intake Power", intakePower);
        telemetry.update();
    }
}