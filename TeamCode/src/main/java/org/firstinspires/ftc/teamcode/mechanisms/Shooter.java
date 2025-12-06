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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;

@Config
public class Shooter {
    private DcMotorEx shooterMotor;
    private DcMotorEx turretMotor;
//    private Limelight3A limelight;

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

    public static double P = 30; ///30, 5, 1, 0 for no variable velocity. Current values are in the works for variable velocity
    public static double I = 4;
    public static double D = 2;
    public static double F = 0;
    public PIDFCoefficients originalPIDF; // 10, 3, 0, 0

    //5.5:1 turret rev

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        // Configure initial settings
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        originalPIDF = shooterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(P, I, D, F);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setVelocity(0);

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

    public double getVoltage() { return shooterMotor.getCurrent(CurrentUnit.AMPS); }


    public void stop() {
        shooterMotor.setPower(0);
    }

//

    public void setTurretTarget(double angle) { //Positive is counter-clockwise
        //Angle to tick conversion factor: 122/15 or 8.13333333
        turretMotor.setTargetPosition((int) (AngleUnit.normalizeDegrees(angle) * 8.13333333333));
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.8);
    }

    public void setTurretMode(DcMotor.RunMode mode) {
        turretMotor.setMode(mode);
    }

    public void setTurretVelocity(int vel, double power) {
        turretMotor.setVelocity(vel);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setPower(power);
    }

    public void setTurretPower(double power) {
        turretMotor.setPower(power);
    }

    public int getTurretPos() {
        return turretMotor.getCurrentPosition();
    }

    public int getTurretTargetPos() {
        return turretMotor.getTargetPosition();
    }

    public DcMotor.RunMode getTurretMode() {
        return turretMotor.getMode();
    }

    public double getTurretVelocity() {
        return turretMotor.getVelocity();
    }


    public static double clamp(double low, double val, double high) {
        if (low > val) {
            return low;
        } else return Math.min(high, val);
    }

}