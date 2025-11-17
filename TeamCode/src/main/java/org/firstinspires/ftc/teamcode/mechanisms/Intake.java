package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private static DcMotorEx intakeMotor;

    private static AnalogInput intakeSensor;
    private static final double intakeDistanceTolerance = 2;
    private static int intakeVelocity = 2000;
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setPower(0);

//        intakeSensor = hardwareMap.get(AnalogInput.class, "intakeSensor");
    }

//    public boolean isBallInIntake() {
//        return (((intakeSensor.getVoltage() / 3.3) * 1000) > 800);
//    }

    // Control the continuous rotation servo
    public void setPower(double power) {
        /*
        final double SLEW_RATE_VELOCITY = 200; // max ticks/sec change per loop
        double currentVelocity = intakeMotor.getVelocity();
        double delta = power > 0 ? intakeVelocity : -intakeVelocity - currentVelocity;

        if (power != 0) {
            // Clamp the change
            delta = Math.max(-SLEW_RATE_VELOCITY, Math.min(delta, SLEW_RATE_VELOCITY));

            intakeMotor.setVelocity(currentVelocity + delta);
        }
         */

        if (power > 0)
            intakeMotor.setVelocity(intakeVelocity);
        else if (power < 0)
            intakeMotor.setVelocity(-intakeVelocity);
        else
            intakeMotor.setVelocity(0);
    }

    public void stop() {
        intakeMotor.setPower(0);}

}
