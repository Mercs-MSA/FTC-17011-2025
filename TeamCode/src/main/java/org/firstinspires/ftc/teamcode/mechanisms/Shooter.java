package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {

    private DcMotor shooterMotor;
    private Servo shooterServo1, shooterServo2;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        shooterServo1 = hardwareMap.get(Servo.class, "shooterServo1");
        shooterServo2 = hardwareMap.get(Servo.class, "shooterServo2");

        // Configure initial settings
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setPower(0);

        shooterServo1.setPosition(0);
        shooterServo2.setPosition(0);
    }

    // Methods to control shooter
    public void setMotorPower(double power) {
        shooterMotor.setPower(power);
    }

    public void setServoPosition1(double pos1) {
        shooterServo1.setPosition(pos1);
    }

    public void setServoPosition2(double pos2) {
        shooterServo1.setPosition(pos2);
    }

    public void stop() {
        shooterMotor.setPower(0);
    }
}
