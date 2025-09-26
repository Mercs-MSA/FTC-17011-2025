package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {

    private DcMotor shooterMotorLeft, shooterMotorRight;
    private Servo shooterServoYaw, shooterServoPitch;



    private enum ShooterState {TRACKING, LOCKED, SHOOTING}

    public Shooter(HardwareMap hardwareMap) {


        shooterMotorLeft = hardwareMap.get(DcMotor.class, "shooterMotorLeft");
        shooterMotorRight = hardwareMap.get(DcMotor.class, "shooterMotorRight");
        shooterServoYaw = hardwareMap.get(Servo.class, "shooterServoYaw");
        shooterServoPitch = hardwareMap.get(Servo.class, "shooterServoPitch");

        // Configure initial settings
        shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotorLeft.setPower(0);
        shooterMotorRight.setPower(0);

        shooterServoYaw.setPosition(0);
        shooterServoPitch.setPosition(0);
    }

    // Methods to control shooter
    public void setMotorPower(double power) {
        shooterMotorLeft.setPower(power);
        shooterMotorRight.setPower(power);
    }

    public void setServoPosition1(double pos1) {
        shooterServoYaw.setPosition(pos1);
    }

    public void setServoPosition2(double pos2) {
        shooterServoYaw.setPosition(pos2);
    }

    public void stop() {
        shooterMotorLeft.setPower(0);
        shooterMotorRight.setPower(0);
    }

    public void runShooter() {

    }

    public void shootArtifact() {

    }
}
