package org.firstinspires.ftc.teamcode.mechanisms;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Teleop;

@Config
public class Transfer {
    private DcMotorEx transferMotor;
    private Servo transferGate;

    public static int transferVelocity = 4000;
    public static double openPosition = 0;
    public static double closedPosition = 1;

    public Transfer(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        transferMotor.setDirection(DcMotorEx.Direction.REVERSE);
        transferMotor.setPower(0);

        transferGate = hardwareMap.get(Servo.class, "transferGate");
        transferGate.setPosition(0);
    }


    public void setPower(double power) {
        if (power > 0)
            transferMotor.setVelocity(transferVelocity);
        else if (power < 0)
            transferMotor.setVelocity(-transferVelocity);
        else
            transferMotor.setVelocity(0);
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
