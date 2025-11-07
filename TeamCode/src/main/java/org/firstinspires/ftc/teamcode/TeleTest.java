package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Spindex Calibration TeleOp", group = "Calibration")
public class TeleTest extends OpMode {

    private DcMotorEx spindexMotor;

    @Override
    public void init() {
        spindexMotor = hardwareMap.get(DcMotorEx.class, "spindexMotor");

        spindexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Spindex Calibration Initialized");
        telemetry.addLine("Use left stick (Y) or triggers to rotate.");
        telemetry.addLine("Press A to reset encoder.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double power = 0.0;

        // Manual control: left stick up/down
        power = -gamepad1.left_stick_y;

        // Fine control: right trigger forward, left trigger backward
        power += (gamepad1.right_trigger - gamepad1.left_trigger) * 0.5;

        // Limit power range
        power = Math.max(-1.0, Math.min(1.0, power));

        spindexMotor.setPower(power);

        // Reset encoder with A button
        if (gamepad1.a) {
            spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Telemetry output
        telemetry.addData("Power", "%.2f", power);
        telemetry.addData("Encoder Position", spindexMotor.getCurrentPosition());
        telemetry.addLine("Press A to reset encoder.");
        telemetry.addLine("Rotate one full revolution, then record this value.");
        telemetry.update();
    }

    @Override
    public void stop() {
        spindexMotor.setPower(0);
    }
}
