package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotorEx intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        // Currently: NEGATIVE power = intake.
        // We want:  POSITIVE power = intake.
        // So flip direction once here and only use positive power in TeleOp.
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setPower(0);
    }

    /**
     * power > 0 : intake (into robot)
     * power < 0 : outtake (backwards)
     * power = 0 : stop
     */
    public void setPower(double power) {
        // Clamp for safety
        if (power > 1.0) power = 1.0;
        if (power < -1.0) power = -1.0;

        intakeMotor.setPower(power);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}