package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class Spindex {
    private DcMotor spindexMotor;
    private ColorSensor spinColorF; //Closest to wheel
    private ColorSensor spinColorR; //Right of the wheel
    private ColorSensor spinColorL; //Left of the wheel

    private enum sensorStates {
        EMPTY,
        PURPLE,
        GREEN
    }

    private sensorStates FState = sensorStates.EMPTY;
    private sensorStates RState = sensorStates.EMPTY;
    private sensorStates LState = sensorStates.EMPTY;

    private Servo spindexServo;

    public Spindex(HardwareMap hardwareMap) {
        spindexMotor = hardwareMap.get(DcMotor.class, "spindexMotor");
        spinColorF = hardwareMap.get(ColorSensor.class, "spindexColor");
        spinColorR = hardwareMap.get(ColorSensor.class, "spindexColor");
        spinColorL = hardwareMap.get(ColorSensor.class, "spindexColor");
        spindexServo = hardwareMap.get(Servo.class, "spindexServo");

        spindexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexMotor.setPower(0);

        spindexServo.setPosition(0);
    }

    // Motor control
    public void setMotorPower(double power) {
        spindexMotor.setPower(power);
    }

    // Servo control
    public void setServoPosition(double position) {
        spindexServo.setPosition(position);
    }

    // Detects ball color: "Purple", "Green", or "Unknown"
//    public String detectBallColor() {
//        float r = spindexColor.red();
//        float g = spindexColor.green();
//        float b = spindexColor.blue();
//
//        if (r > 100 && b > 100 && g < 80) {
//            return "Purple";
//        } else if (g > 120 && r < 100 && b < 100) {
//            return "Green";
//        } else {
//            return "Unknown";
//        }
//    }

    public int colorDetect() {
        int positionsFilled = 0;
        float r = spinColorF.red();
        float g = spinColorF.green();
        float b = spinColorF.blue();

        FState = (r > 100 && b > 100 && g < 80) ? sensorStates.GREEN : (g > 120 && r < 100 && b < 100) ? sensorStates.PURPLE : sensorStates.EMPTY;
        positionsFilled += (!FState.equals(sensorStates.EMPTY)) ? 1 : 0;

        r = spinColorL.red();
        g = spinColorL.green();
        b = spinColorL.blue();

        LState = (r > 100 && b > 100 && g < 80) ? sensorStates.GREEN : (g > 120 && r < 100 && b < 100) ? sensorStates.PURPLE : sensorStates.EMPTY;
        positionsFilled += (!LState.equals(sensorStates.EMPTY)) ? 1 : 0;

        r = spinColorR.red();
        g = spinColorR.green();
        b = spinColorR.blue();

        RState = (r > 100 && b > 100 && g < 80) ? sensorStates.GREEN : (g > 120 && r < 100 && b < 100) ? sensorStates.PURPLE : sensorStates.EMPTY;
        positionsFilled += (!RState.equals(sensorStates.EMPTY)) ? 1 : 0;

        return positionsFilled;
    }
    public sensorStates getColorF() {return FState;}
    public sensorStates getColorL() {return LState;}
    public sensorStates getColorR() {return RState;}
}
