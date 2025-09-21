package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GeneralConstants;

public class Spindex {
    private static DcMotor spindexMotor;
    private static ColorSensor spindexColorBack; //Closest to wheel
    private static ColorSensor spindexColorRight; //Right of the wheel
    private static ColorSensor spindexColorLeft; //Left of the wheel



    private GeneralConstants.artifactColors spindexColorBackState;
    private GeneralConstants.artifactColors spindexColorRightState;
    private GeneralConstants.artifactColors spindexColorLeftState;

    private static CRServo spindexTransferServo;
    private static Servo spindexGateServo;

    public static int numOfArtifactsInRobot = 0;

    private static final double spindexGateOpenPosition = 0.0;
    private static final double spindexGateClosedPosition = 0.0;

    public Spindex(HardwareMap hardwareMap) {
        spindexMotor = hardwareMap.get(DcMotor.class, "spindexMotor");

        spindexTransferServo = hardwareMap.get(CRServo.class, "spindexTransferServo");
        spindexGateServo = hardwareMap.get(Servo.class, "spindexGateServo");

        spindexColorBack = hardwareMap.get(ColorSensor.class, "spindexColorF");
        spindexColorRight = hardwareMap.get(ColorSensor.class, "spindexColorR");
        spindexColorLeft = hardwareMap.get(ColorSensor.class, "spindexColorL");

        spindexColorBackState = GeneralConstants.artifactColors.EMPTY;
        spindexColorRightState = GeneralConstants.artifactColors.EMPTY;
        spindexColorLeftState = GeneralConstants.artifactColors.EMPTY;

        spindexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexMotor.setPower(0);

        spindexTransferServo.setPower(0);
    }

    private static void openSpindexGate() {
        spindexGateServo.setPosition(spindexGateOpenPosition);
    }

    private static void closeSpindexGate () {
        spindexGateServo.setPosition(spindexGateClosedPosition);
    }

    public static void runSpindex() {
        spindexMotor.setPower(1);
    }

    public static void stopSpindex() {
        spindexMotor.setPower(0);
    }

    public static void runTransferWheel() {
        spindexTransferServo.setPower(1);
    }

    public static void stopTransferWheel() {
        spindexTransferServo.setPower(0);
    }


    public void runSpindexToColor(GeneralConstants.artifactColors targetColor) {
        if (targetColor == GeneralConstants.artifactColors.EMPTY) {
            throw new IllegalArgumentException("Target color cannot be empty");
        } else {
            for (int i = 0; i < 2; i++) {
                updateSpinColorSensors();
                if (spindexColorBackState == targetColor)
                    return;
                runSpindexToNextArtifact();
            }
            updateSpinColorSensors();
        }
    }

    public void runSpindexToNextArtifact() {
        final int encoderTicksForNextArtifact = 300; //To Be Confirmed

        spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        spindexMotor.setTargetPosition((encoderTicksForNextArtifact) + spindexMotor.getCurrentPosition());
        spindexMotor.setPower(1);

        spindexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static GeneralConstants.artifactColors getColor(ColorSensor colorSensor) {
        float r = colorSensor.red();
        float g = colorSensor.green();
        float b = colorSensor.blue();

        if (r > 100 && b > 100 && g < 80)
            return GeneralConstants.artifactColors.GREEN;
        else if (g > 120 && r < 100 && b < 100)
            return GeneralConstants.artifactColors.PURPLE;
        else
            return GeneralConstants.artifactColors.EMPTY;
    }

    public void updateSpinColorSensors() {

        spindexColorBackState = getColor(spindexColorBack);
        spindexColorLeftState = getColor(spindexColorLeft);
        spindexColorRightState = getColor(spindexColorRight);

        numOfArtifactsInRobot += (spindexColorBackState.equals(GeneralConstants.artifactColors.EMPTY)) ? 0 : 1;
        numOfArtifactsInRobot += (spindexColorLeftState.equals(GeneralConstants.artifactColors.EMPTY)) ? 0 : 1;
        numOfArtifactsInRobot += (spindexColorRightState.equals(GeneralConstants.artifactColors.EMPTY)) ? 0 : 1;
    }

    public int getNumOfArtifactsInRobot() {
        updateSpinColorSensors();
        return numOfArtifactsInRobot;
    }
}
