package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {

    private DcMotor shooterMotor1;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotor.class, "launcher");

        // Configure initial settings
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor1.setPower(0);
    }

    // Methods to control shooter
    public void setPower(double power) {
        shooterMotor1.setPower(power);
    }

    public double getPower() {return shooterMotor1.getPower();}

    public void stop() {
        shooterMotor1.setPower(0);
    }
}
