package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private CRServo right;
    private CRServo left;


    public Intake(HardwareMap hardwareMap) {
        right = hardwareMap.get(CRServo.class, "right_feeder");
        left = hardwareMap.get(CRServo.class, "left_feeder");

        right.setPower(0);
        left.setPower(0);
    }

    // Control the continuous rotation servo
    public void setPower(double power) {
        right.setPower(power);
        left.setPower(power);
    }

    public void stop() {
        right.setPower(0);
        left.setPower(0);
    }

    public double getRightPower() {
        return right.getPower();
    }

    public double getLeftPower() {
        return left.getPower();
    }
}
