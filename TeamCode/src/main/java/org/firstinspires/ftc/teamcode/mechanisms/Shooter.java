package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private final DcMotorEx shooterMotor;

    // private final DcMotorEx shooterMotorLeft;
    // private final DcMotorEx shooterMotorRight;

    public Shooter(HardwareMap hardwareMap) {

        // --- SINGLE MOTOR SETUP ---
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setVelocity(0);


        /*
        shooterMotorLeft  = hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "shooterMotorRight");

        shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterMotorRight.setDirection(DcMotor.Direction.REVERSE);

        shooterMotorLeft.setVelocity(0);
        shooterMotorRight.setVelocity(0);
        */
    }


    public void setShooterPower(double power) {
        shooterMotor.setPower(power);
    }

    public void setMotorVelocity(double velocity) {
        shooterMotor.setVelocity(velocity);
    }

    public double getVelocity() {
        return shooterMotor.getVelocity();
    }

    public void stop() {
        shooterMotor.setPower(0);
    }

    /*
    public void setShooterPower(double power) {
        shooterMotorLeft.setPower(power);
        shooterMotorRight.setPower(power);
    }

    public void setMotorVelocity(double velocity) {
        shooterMotorLeft.setVelocity(velocity);
        shooterMotorRight.setVelocity(velocity);
    }

    public double getLeftVelocity() { return shooterMotorLeft.getVelocity(); }
    public double getRightVelocity() { return shooterMotorRight.getVelocity(); }

    public void stop() {
        shooterMotorLeft.setPower(0);
        shooterMotorRight.setPower(0);
    }
    */
}