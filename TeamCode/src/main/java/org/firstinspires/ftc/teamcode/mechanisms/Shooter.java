package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;

@Config
public class Shooter {
    private DcMotorEx shooterMotor;
    private DcMotorEx shooterTurretMotor;

    private static double goalAngle = 0;
    private static double currentAngle = 0;
    private static double pipeline = 0;

    private static double goalRange = 4;

    //5.5:1 turret rev




    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        // Configure initial settings
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setVelocityPIDFCoefficients(20, 3, 0, 5);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotor.setVelocity(0);

    }

    public double getRpm() {
        // getVelocity() returns ticks/second; convert to RPM
        return (shooterMotor.getVelocity() * 60.0) / 28.0;
    }

    public PIDFCoefficients getPID() {
        return (shooterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
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

    public double getVelocity() {
        return shooterMotor.getVelocity();
    }


    public void stop() {
        shooterMotor.setPower(0);
    }
}