package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Teleop;

@Config
public class Spindex {
    public DcMotorEx spindexMotor;
    public ColorRangeSensor spindexColorBack;
    public ColorRangeSensor spindexColorRight;
    public ColorRangeSensor spindexColorLeft;

    public static double currentSpindexPosition = 0;
    public static double spindexPositionFromAuto = 0;


    private GeneralConstants.colorSensorStates spindexColorBackState;
    private GeneralConstants.colorSensorStates spindexColorRightState;
    private GeneralConstants.colorSensorStates spindexColorLeftState;

    private static CRServo spindexTransferServo;

    public static double spindexMotorVelocity = 600;

    public GeneralConstants.colorSensorStates targetColor = GeneralConstants.colorSensorStates.EMPTY;


    public Spindex(HardwareMap hardwareMap) {
        spindexMotor = hardwareMap.get(DcMotorEx.class, "spindexMotor");

        spindexTransferServo = hardwareMap.get(CRServo.class, "spindexTransferServo");
        spindexColorBack = hardwareMap.get(ColorRangeSensor.class, "spindexColorB");
        spindexColorRight = hardwareMap.get(ColorRangeSensor.class, "spindexColorR");
        spindexColorLeft = hardwareMap.get(ColorRangeSensor.class, "spindexColorL");

        spindexColorBackState = GeneralConstants.colorSensorStates.EMPTY;
        spindexColorRightState = GeneralConstants.colorSensorStates.EMPTY;
        spindexColorLeftState = GeneralConstants.colorSensorStates.EMPTY;

        spindexMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        spindexMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentSpindexPosition = 0.00;
        spindexMotor.setTargetPosition((int) (currentSpindexPosition));
        spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexMotor.setVelocity(spindexMotorVelocity);

        spindexTransferServo.setDirection(CRServo.Direction.REVERSE);
        spindexTransferServo.setPower(0);
    }

    public boolean isSpindexMoving() {
        return !(Math.abs(currentSpindexPosition - spindexMotor.getCurrentPosition()) < 5);
    }


    public void changeCurrentPositionBy(double positionChange) {
        currentSpindexPosition += positionChange;
        spindexMotor.setTargetPosition((int) (Math.round(currentSpindexPosition)));
        spindexMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spindexMotor.setVelocity(spindexMotorVelocity);
    }

    public void moveSpindexToZero() {
        currentSpindexPosition = 0;
        spindexMotor.setTargetPosition((int) (Math.round(currentSpindexPosition)));
        spindexMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spindexMotor.setVelocity(800);
    }
    public void resetSpindexEncoder() {
        currentSpindexPosition = 0;
        spindexMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runSpindexToTransferThird() {
        if (currentSpindexPosition % Teleop.spindexThirdRevolution != 0) {
            changeCurrentPositionBy(Teleop.spindexThirdRevolution / 2.0);
        }
    }


    public void stopSpindex() {
        spindexMotor.setVelocity(0);
    }


    public void runTransferWheel() {
        spindexTransferServo.setPower(1);
    }

    public void runTransferWheelReverse() {
        spindexTransferServo.setPower(-1);
    }


    public void stopTransferWheel() {
        spindexTransferServo.setPower(0);
    }

    public String getColorRaw(ColorRangeSensor colorSensor) {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        return "R: " + r + " G: " + g + " B: " + b;
    }

    ///IMPORTANT: UNTUNED METHOD
    public String getColor(ColorRangeSensor colorRangeSensor) {
        int r = colorRangeSensor.red();
        int g = colorRangeSensor.green();
        int b = colorRangeSensor.blue();

        // White = all channels high and close together
        int max = Math.max(r, Math.max(g, b));
        int min = Math.min(r, Math.min(g, b));
        if (max > 100 && (max - min) < 25) {
            return "EMPTY";
        }

        // Purple = red + blue high, green low
        if (r > 80 && b > 80 && g < 50) {
            return "P";
        }

        // Green = green dominant
        if (g > r && g > b && g > 80) {
            return "G";
        }

        return "unknown";
    }
}
