package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;

public class Spindex {
    private static DcMotorEx spindexMotor;
    private static ColorRangeSensor spindexColorBack; //Closest to wheel
    private static ColorRangeSensor spindexColorRight; //Right of the wheel
    private static ColorRangeSensor spindexColorLeft; //Left of the wheel



    private GeneralConstants.artifactColors spindexColorBackState;
    private GeneralConstants.artifactColors spindexColorRightState;
    private GeneralConstants.artifactColors spindexColorLeftState;

    private static CRServo spindexTransferServo;
    private static Servo spindexGateServo;

    private static int numOfArtifactsInRobot = 0;

    private static final double spindexGateOpenPosition = 0.0;
    private static final double spindexGateClosedPosition = 0.0;

    public Spindex(HardwareMap hardwareMap) {
        spindexMotor = hardwareMap.get(DcMotorEx.class, "spindexMotor");

        spindexTransferServo = hardwareMap.get(CRServo.class, "spindexTransferServo");
        spindexGateServo = hardwareMap.get(Servo.class, "spindexGateServo");

        spindexColorBack = hardwareMap.get(ColorRangeSensor.class, "spindexColorF");
        spindexColorRight = hardwareMap.get(ColorRangeSensor.class, "spindexColorR");
        spindexColorLeft = hardwareMap.get(ColorRangeSensor.class, "spindexColorL");

        spindexColorBackState = GeneralConstants.artifactColors.EMPTY;
        spindexColorRightState = GeneralConstants.artifactColors.EMPTY;
        spindexColorLeftState = GeneralConstants.artifactColors.EMPTY;

        spindexMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        spindexMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        spindexMotor.setPower(0);

        spindexTransferServo.setPower(0);
    }

    public void openSpindexGate() {
        spindexGateServo.setPosition(spindexGateOpenPosition);
    }

    public void closeSpindexGate () {
        spindexGateServo.setPosition(spindexGateClosedPosition);
    }

    public void runSpindex() {
        spindexMotor.setPower(1);
    }

    public void stopSpindex() {
        spindexMotor.setPower(0);
    }

    public void runTransferWheel() {
        spindexTransferServo.setPower(1);
    }

    public void stopTransferWheel() {
        spindexTransferServo.setPower(0);
    }


    public void runSpindexToColor(GeneralConstants.artifactColors targetColor) {
        if (targetColor.equals(GeneralConstants.artifactColors.EMPTY)) {
            throw new IllegalArgumentException("Target color cannot be empty");
        } else {
            for (int i = 0; i < 2; i++) {
                updateSpinColorSensors();
                if (spindexColorBackState.equals(targetColor))
                    return;
                else if (spindexColorRightState.equals(targetColor))
                    runSpindexToNextArtifact(false);
                else
                    runSpindexToNextArtifact(true);
            }
            updateSpinColorSensors();
        }
    }

    private void runSpindexToNextArtifact(boolean reverse) {
//        spindexMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

//        spindexMotor.setTargetPosition((encoderTicksForNextArtifact) + spindexMotor.getCurrentPosition());
        while (getColor(spindexColorBack).equals(GeneralConstants.artifactColors.EMPTY)) {
            double velocity = (reverse ? -1 : 1) * spindexColorBack.getDistance(DistanceUnit.INCH) * 10;
            spindexMotor.setVelocity(velocity);
            if (!getColor(spindexColorLeft).equals(GeneralConstants.artifactColors.EMPTY) || !getColor(spindexColorRight).equals(GeneralConstants.artifactColors.EMPTY))
                return;
        }

//        spindexMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public GeneralConstants.artifactColors getColor(ColorRangeSensor colorSensor) {
        float r = colorSensor.red();
        float g = colorSensor.green();
        float b = colorSensor.blue();

        if (r > 100 && b > 100 && g < 80 && colorSensor.getDistance(DistanceUnit.INCH) < 1)
            return GeneralConstants.artifactColors.GREEN;
        else if (g > 120 && r < 100 && b < 100 && colorSensor.getDistance(DistanceUnit.INCH) < 1)
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
