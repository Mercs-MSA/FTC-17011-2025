package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private static CRServo intakeServo;
    public Intake(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        intakeServo.setDirection(CRServo.Direction.REVERSE);
        intakeServo.setPower(0.0);
    }

    // Control the continuous rotation servo
    public void setPower(double power) {
        intakeServo.setPower(power);
    }

    public void stop() {intakeServo.setPower(0);}

}
