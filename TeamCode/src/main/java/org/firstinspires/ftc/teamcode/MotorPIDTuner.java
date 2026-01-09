package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp
public class MotorPIDTuner extends OpMode {

    private DcMotorEx motor;
    public static double targetVelocity = 1967;

    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.0;
    private double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    PIDFCoefficients curPIDFCoefficients;

    public int stepIndex = 1;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "shooterMotor"); //CHANGE MOTOR HERE
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        curPIDFCoefficients = new PIDFCoefficients(P, I, D, F);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER, curPIDFCoefficients);

        telemetry.addLine("Initalized");
    }

    @Override
    public void loop() {

        if (gamepad1.right_trigger > 0.367) {
            motor.setVelocity(targetVelocity);
        }

        curPIDFCoefficients = new PIDFCoefficients(P, I, D, F);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER, curPIDFCoefficients);

        telemetry.addData("P: ", P);
        telemetry.addData("I: ", I);
        telemetry.addData("D: ", D);
        telemetry.addData("F: ", F);

        telemetry.addData("Target Velocity: ", targetVelocity);
        telemetry.addData("Current Velocity: ", motor.getVelocity());
        telemetry.addData("Error Velocity: ", targetVelocity - motor.getVelocity());


    }

}
