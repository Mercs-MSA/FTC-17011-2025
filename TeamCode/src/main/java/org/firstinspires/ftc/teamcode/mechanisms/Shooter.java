package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;

public class Shooter {
    private DcMotorEx shooterMotorLeft, shooterMotorRight;

    private static int goalAngle = 0;
    private static int currentAngle = 0;
    private static int pipeline = 0;

    private static double goalRange = 4;





    public Shooter(HardwareMap hardwareMap) {
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "shooterMotorRight");

        // Configure initial settings
        shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotorRight.setDirection(DcMotor.Direction.REVERSE);
        shooterMotorLeft.setDirection(DcMotor.Direction.FORWARD);

        shooterMotorRight.setVelocity(0);
        shooterMotorLeft.setVelocity(0);

    }

    public double getRpm() {
        // getVelocity() returns ticks/second; convert to RPM
        return (shooterMotorRight.getVelocity() * 60.0) / 28.0;
    }



    public void setShooterPower(double power) {
        shooterMotorLeft.setPower(power);
        shooterMotorRight.setPower(power);
    }

    // Methods to control shooter
    public void setMotorVelocity(double velocity) {
        shooterMotorLeft.setVelocity(velocity);
        shooterMotorRight.setVelocity(velocity);
    }


    public double getRightVelocity() {
        return shooterMotorRight.getVelocity();
    }
    public double getLeftVelocity() {
        return shooterMotorLeft.getVelocity();
    }

    public void stop() {
        shooterMotorLeft.setPower(0);
        shooterMotorRight.setPower(0);
    }
}
