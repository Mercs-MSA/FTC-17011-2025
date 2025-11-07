package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private static DcMotorEx intakeMotor;
    private static int intakeVelocity = 400;
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setPower(0);
    }

    // Control the continuous rotation servo
    public void setPower(double power) {
        intakeMotor.setPower(power);
        if (power > 0)
            intakeMotor.setVelocity(400);
        else if (power < 0)
            intakeMotor.setVelocity(-400);
        else
            intakeMotor.setVelocity(0);
    }

    public void stop() {
        intakeMotor.setPower(0);}

}
