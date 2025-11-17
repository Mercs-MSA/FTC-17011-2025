package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private final DcMotorEx shooterMotorLeft;
    private final DcMotorEx shooterMotorRight;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotorLeft  = hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "shooterMotorRight");

        shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set directions so positive velocity = wheels spin to shoot OUT
        shooterMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterMotorRight.setDirection(DcMotor.Direction.REVERSE);

        shooterMotorLeft.setVelocity(0);
        shooterMotorRight.setVelocity(0);
    }

    public void setShooterPower(double power) {
        shooterMotorLeft.setPower(power);
        shooterMotorRight.setPower(power);
    }

    public void setMotorVelocity(double velocity) {
        shooterMotorLeft.setVelocity(velocity);
        shooterMotorRight.setVelocity(velocity);
    }

    public double getLeftVelocity() {
        return shooterMotorLeft.getVelocity();
    }

    public double getRightVelocity() {
        return shooterMotorRight.getVelocity();
    }

    public void stop() {
        shooterMotorLeft.setPower(0);
        shooterMotorRight.setPower(0);
    }
}