package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;

@Config
@TeleOp
public class MotorPIDTuner extends OpMode {
    private static Telemetry myTelem;
    private FtcDashboard dash;
    private ElapsedTime timer;


    private DcMotorEx motor;
    private Shooter shooter;
    private Transfer transfer;
    public static double targetVelocity = 1967;

    public static double P = 80;
    public static double I = 0;
    public static double D = 1;
    public static double F = 15;
    private double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    PIDFCoefficients curPIDFCoefficients;

    public int stepIndex = 1;

    private boolean desiredVelocityReached() {
        return (shooter.getVelocity() > 1967 * .98);
    }

    @Override
    public void init() {
//        motor = hardwareMap.get(DcMotorEx.class, "shooterMotor"); //CHANGE MOTOR HERE
//        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//        motor.setDirection(DcMotor.Direction.FORWARD);
//        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        curPIDFCoefficients = new PIDFCoefficients(P, I, D, F);
//        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER, curPIDFCoefficients);
        dash = FtcDashboard.getInstance();
        myTelem = new MultipleTelemetry(dash.getTelemetry(), telemetry);

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);

        myTelem.addLine("Initalized");
        myTelem.update();
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0.367) {
            shooter.setMotorVelocity(targetVelocity);
        } else {
            shooter.setMotorVelocity(0);
        }

        if (gamepad1.left_trigger > 0.367 && desiredVelocityReached()) {
            transfer.setPower(.8);
        } else {
            transfer.setPower(0);
        }

        if (gamepad1.dpad_down) {
            timer.reset();
        }

        curPIDFCoefficients = new PIDFCoefficients(P, I, D, F);
        shooter.setShooterPIDF(P, I, D, F);

        myTelem.addData("P: ", P);
        myTelem.addData("I: ", I);
        myTelem.addData("D: ", D);
        myTelem.addData("F: ", F);

        myTelem.addData("Target Velocity: ", targetVelocity);
        myTelem.addData("Current Velocity: ", shooter.getVelocity());
        myTelem.addData("Error Velocity: ", targetVelocity - shooter.getVelocity());
        myTelem.addData("Timer: ", timer.time());

        myTelem.addData("Upper: ", 2200);
        myTelem.addData("Lower: ", 0);
        myTelem.update();
    }

}
