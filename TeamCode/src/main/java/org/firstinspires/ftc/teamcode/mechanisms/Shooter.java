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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;

public class Shooter {
    private DcMotorEx shooterMotor;
    private DcMotorEx turretMotor;
    private Limelight3A limelight;

    private static int goalAngle = 0;
    private static int currentAngle = 0;
    private static int pipeline = 0;

    private static double goalRange = 4;

    public static double rightMaxPosition = 500;
    public static double leftMaxPosition = 500;

    private enum TURRET_STATE {
        ZEROED,
        ZEROING,
        AIMING_NO_TAG,
        AIMING_TO_TAG
    }

    TURRET_STATE turretState = TURRET_STATE.ZEROED;

    //5.5:1 turret rev

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Configure initial settings
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setVelocityPIDFCoefficients(20, 3, 0, 5);
        turretMotor.setVelocityPIDFCoefficients(1, 0, 0, 0);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shooterMotor.setVelocity(0);

        limelight.start();
        limelight.pipelineSwitch(0); //Pipeline 0 = Blue Tag (ID 20), Pipeline 1 = Red Tag (ID 24)
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

    public LLResult getLLResult() {
        LLResult llResult = limelight.getLatestResult();
        return llResult;
    }

    public void setTurretTarget(int pos) {
        turretMotor.setTargetPosition(pos);
    }

    public void setTurretMode(DcMotor.RunMode mode) {
        turretMotor.setMode(mode);
    }

    public void setTurretVelocity(int vel) {
        turretMotor.setVelocity(vel);
    }

    public void setTurretPower(double power) {
    }

    public int getTurretPos() {
        return turretMotor.getCurrentPosition();
    }


    public void lockOn() {
        double tx = 0;
        if (getLLResult().isValid())
            tx = getLLResult().getTx();
        else
            return;

        if (tx > .2) {
            turretMotor.setVelocity(11);
        }
    }
}