package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleTest extends OpMode {
    private Servo transferGate;
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private DcMotorEx turretMotor;
    private DcMotorEx shooterMotor;

    private double turretPower = 0;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        turretMotor.setPower(0);

//        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        turretMotor.setTargetPosition(0);
//        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        turretMotor.setPower(0);

        transferGate = hardwareMap.get(Servo.class, "transferGate");

        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        shooterMotor.setPower(0);

        telemetry.addLine("Spindex Calibration Initialized");
        telemetry.addLine("Use left stick (Y) or triggers to rotate.");
        telemetry.addLine("Press A to reset encoder.");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        shooterMotor.setPower(1);
        shooterMotor.setVelocity(0);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            intakeMotor.setPower(1);
            transferMotor.setPower(.7);
        } else if (gamepad1.left_bumper) {
            intakeMotor.setPower(-1);
            transferMotor.setPower(-.7);
        } else {
            intakeMotor.setPower(0);
            transferMotor.setPower(0);
        }

        if (gamepad1.right_trigger > 0.5) {
            turretMotor.setPower(0);
        }

        if (gamepad1.circle) {
            transferGate.setPosition(1);
        } else {
            transferGate.setPosition(0);
        }

        if (gamepad1.dpad_up) {
            shooterMotor.setVelocity(1200);
        } else {
            shooterMotor.setVelocity(0);
        }

        turretPower = gamepad1.right_trigger - gamepad1.left_trigger;

        if (Math.abs(turretPower) > 0) {
            turretMotor.setPower(turretPower);
            turretMotor.setTargetPosition(turretMotor.getCurrentPosition() + (int) (25 * (turretPower)));
        }

        // Telemetry output
        telemetry.addData("Shooter velocity: ", shooterMotor.getVelocity());
        telemetry.addData("Turret Position: ", turretMotor.getCurrentPosition());
        telemetry.update();
    }
}
