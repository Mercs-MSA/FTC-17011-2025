package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {

    private DcMotorEx shooterMotorLeft, shooterMotorRight;
    private CRServo shooterServoYaw;
    private Servo shooterServoPitch;

    private static int goalAngle = 0;
    private static int currentAngle = 0;

    public int getCurrentAngle() {
        return currentAngle;
    }

    public int getGoalAngle() {
        return goalAngle;
    }


    public Shooter(HardwareMap hardwareMap) {
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "shooterMotorRight");
        shooterServoYaw = hardwareMap.get(CRServo.class, "shooterServoYaw");
        shooterServoPitch = hardwareMap.get(Servo.class, "shooterServoPitch");

        // Configure initial settings
        shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotorRight.setDirection(DcMotor.Direction.REVERSE);
        shooterMotorLeft.setDirection(DcMotor.Direction.REVERSE);

        shooterMotorLeft.setPower(0);
        shooterMotorRight.setPower(0);

        shooterServoYaw.setPower(0);
        shooterServoPitch.setPosition(0);
    }

    // Methods to control shooter
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

    public void setTurretYawPower(double p) {
        shooterServoYaw.setPower(p);
    }

//    public void setServoPosition1(double pos1) {
//        shooterServoYaw.setPosition(pos1);
//    }
//
//    public void setServoPosition2(double pos2) {
//        shooterServoYaw.setPosition(pos2);
//    }

    public void stopShooterMotor() {
        shooterMotorLeft.setPower(0);
        shooterMotorRight.setPower(0);
    }

    public void pointAtGoal() {

    }

    public void shootArtifact() {

    }
}
