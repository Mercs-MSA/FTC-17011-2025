package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Transfer {
    private DcMotorEx transferMotor;
    private Servo transferGate;

    public static int transferVelocity = 4000;
    public static double openPosition = .25;
    public static double closedPosition = .40;

    public Transfer(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");

        transferMotor.setDirection(DcMotorEx.Direction.REVERSE);
        transferMotor.setPower(0);

        transferGate = hardwareMap.get(Servo.class, "transferGate");
        transferGate.setPosition(0);
    }



    /**
     * power > 0 : feed note toward shooter
     * power < 0 : run backwards
     * power = 0 : stop
     */
    public void setPower(double power) {
        if (power > 1.0) power = 1.0;
        if (power < -1.0) power = -1.0;
        transferMotor.setPower(power);
    }

    public void stopTransferWheel() {
        transferMotor.setPower(0);}

    public void closeTransferGate() {
        transferGate.setPosition(closedPosition);
    }

    public void openTransferGate() {
        transferGate.setPosition(openPosition);
    }

    public double getTransferPosition() {
        return transferGate.getPosition();
    }
}
