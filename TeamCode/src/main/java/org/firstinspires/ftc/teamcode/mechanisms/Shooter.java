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

    public static double P = 80; ///30, 5, 1, 0 for no variable velocity. Current values are in the works for variable velocity
    public static double I = 0;
    public static double D = 1;
    public static double F = 15;
    public PIDFCoefficients originalPIDF; // 10, 3, 0, 0

    //5.5:1 turret rev

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Configure initial settings
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        shooterMotor.setVelocityPIDFCoefficients(P, I, D, F);
        turretMotor.setPositionPIDFCoefficients(10);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setVelocity(0);

        shooterMotor.setVelocity(0);

//        limelight.start();
//        limelight.pipelineSwitch(0); //Pipeline 0 = Blue Tag (ID 20), Pipeline 1 = Red Tag (ID 24)
    }

    public void setShooterPIDF(double p, double i, double d, double f) {
        shooterMotor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public double getRpm() {
        // getVelocity() returns ticks/second; convert to RPM
        return (shooterMotor.getVelocity() * 60.0) / 28.0;
    }

    public PIDFCoefficients getShooterPID() {
        return (shooterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
    }

    public PIDFCoefficients getTurretPositionalPID() {
        return turretMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
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
        turretMotor.setTargetPosition((int)(angle * 8.13333333333));
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.8);
    }

    public void setTurretTargetShortestPath(double desiredAngleDeg) {
        // normalize desired angle to -180..180 (tolerant: user can pass normalized or raw)
        double desired = AngleUnit.normalizeDegrees(desiredAngleDeg);

        // read current encoder & convert to physical angle (may be >360 or <0 depending on your encoder origin)
        int currentTicks = turretMotor.getCurrentPosition();
        double currentAngleDeg = currentTicks / 8.13333333; // this gives a continuous angle
        // but we only need the physical angle modulo 360 for the shortest-path calculation:
        double currentPhysicalAngle = AngleUnit.normalizeDegrees(currentAngleDeg);

        // shortest-path error (−180..+180)
        double errorDeg = AngleUnit.normalizeDegrees(desired - currentPhysicalAngle);

        // optional: clamp huge jumps (safety), e.g. protect against sensor glitches
//        if (errorDeg > MAX_STEP_DEG) errorDeg = MAX_STEP_DEG;
//        if (errorDeg < -MAX_STEP_DEG) errorDeg = -MAX_STEP_DEG;

        // deadband: if already close, don't re-command (avoids hunting)
        if (Math.abs(errorDeg) < 2) {
            // optionally stop motor / switch mode
            turretMotor.setPower(0.0);
            return;
        }

        // compute tick delta for shortest path and make absolute target = current + delta
        int deltaTicks = (int) Math.round(errorDeg * 8.13333333);
        int targetTicks = currentTicks + deltaTicks;

        // command motor
        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.8); // tune power
    }

    public void setTurretMode(DcMotor.RunMode mode) {
        turretMotor.setMode(mode);
    }

    public void setTurretVelocity(int vel, double power) {
        turretMotor.setVelocity(vel);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        turretMotor.setPower(power);
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



//    public void lockOn() {
//        double tx = 0;
//        if (getLLResult().isValid())
//            tx = getLLResult().getTx();
//        else
//            return;
//
//        if (tx > .2) {
//            turretMotor.setVelocity(11);
//        }
//    }
    public static double clamp(double low, double val, double high) {
        if (low > val) {
            return low;
        } else return Math.min(high, val);
    }

}