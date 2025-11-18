package org.firstinspires.ftc.teamcode.mechanisms;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Teleop;

@Config
public class Transfer {
    public static DcMotorEx transferMotor;

    private static int transferVelocity = 4000;

    public Transfer(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        transferMotor.setDirection(DcMotorEx.Direction.REVERSE);
        transferMotor.setPower(0);

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
}
