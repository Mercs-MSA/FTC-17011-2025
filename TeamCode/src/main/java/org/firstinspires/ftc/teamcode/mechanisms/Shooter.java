package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;

public class Shooter {
    private DcMotorEx shooterMotor;
    private DcMotorEx shooterTurretMotor;

    private static int goalAngle = 0;
    private static int currentAngle = 0;
    private static int pipeline = 0;

    private static double goalRange = 4;

    //5.5:1 turret rev




    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
        // Configure initial settings
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        shooterMotor.setVelocity(0);

    }

    public double getRpm() {
        // getVelocity() returns ticks/second; convert to RPM
        return (shooterMotor.getVelocity() * 60.0) / 28.0;
    }



    public void setShooterPower(double power) {
        shooterMotor.setPower(power);
    }

    // Methods to control shooter
    public void setMotorVelocity(double velocity) {
        shooterMotor.setVelocity(velocity);
    }

    public double getShooterCurrent() {
        return shooterMotor.getCurrent(CurrentUnit.AMPS);
    }

    public double getLeftVelocity() {
        return shooterMotor.getVelocity();
    }


    public void stop() {
        shooterMotor.setPower(0);
    }
}
