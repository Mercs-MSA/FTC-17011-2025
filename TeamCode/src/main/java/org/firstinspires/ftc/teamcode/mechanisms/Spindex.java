package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;

@Config
public class Spindex {
    private static DcMotorEx spindexMotor;
    public ColorRangeSensor spindexColorBack; //Closest to wheel
    public ColorRangeSensor spindexColorRight; //Right of the wheel
    public ColorRangeSensor spindexColorLeft; //Left of the wheel
    public static double spindexVelocity = 600;



    private GeneralConstants.artifactColors spindexColorBackState;
    private GeneralConstants.artifactColors spindexColorRightState;
    private GeneralConstants.artifactColors spindexColorLeftState;

    private static CRServo spindexTransferServo;
    private static Servo spindexGateServo;

    private static int numOfArtifactsInRobot = 0;

    public static double spindexGateOpenPosition = 0.0;
    public static double spindexGateClosedPosition = 0.0;

    public Spindex(HardwareMap hardwareMap) {
        spindexMotor = hardwareMap.get(DcMotorEx.class, "spindexMotor");

        spindexTransferServo = hardwareMap.get(CRServo.class, "spindexTransferServo");
        spindexGateServo = hardwareMap.get(Servo.class, "spindexGateServo");

        spindexColorBack = hardwareMap.get(ColorRangeSensor.class, "spindexColorB");
        spindexColorRight = hardwareMap.get(ColorRangeSensor.class, "spindexColorR");
        spindexColorLeft = hardwareMap.get(ColorRangeSensor.class, "spindexColorL");

        spindexColorBackState = GeneralConstants.artifactColors.EMPTY;
        spindexColorRightState = GeneralConstants.artifactColors.EMPTY;
        spindexColorLeftState = GeneralConstants.artifactColors.EMPTY;

        spindexMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        spindexMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        spindexMotor.setPower(1);
        spindexMotor.setVelocity(0);

        spindexTransferServo.setPower(0);
    }

    public void openSpindexGate() {
        spindexGateServo.setPosition(spindexGateOpenPosition);
    }

    public void closeSpindexGate () {
        spindexGateServo.setPosition(spindexGateClosedPosition);
    }

    public void runSpindex() {
        spindexMotor.setVelocity(spindexVelocity);
    }

    public void initSpindex() {
        spindexMotor.setPower(1);
        spindexMotor.setVelocity(0);
    }

    public void stopSpindex() {
        spindexMotor.setVelocity(0);
    }

    public double getSpindexVelocity() {
        return spindexMotor.getVelocity();
    }

    public double getSpindexPosition() {
        return spindexMotor.getCurrentPosition();
    }

    public void runTransferWheel() {
        spindexTransferServo.setPower(1);
    }

    public void stopTransferWheel() {
        spindexTransferServo.setPower(0);
    }


    public void runSpindexToColor(GeneralConstants.artifactColors targetColor) {
        updateSpinColorSensors();
        if (targetColor.equals(GeneralConstants.artifactColors.EMPTY)) {
            throw new IllegalArgumentException("Target color cannot be empty");
        } else if (!spindexColorRightState.equals(targetColor) && !spindexColorLeftState.equals(targetColor) && !spindexColorBackState.equals(targetColor)) {
            return;
        } else {
            if (spindexColorBackState.equals(targetColor))
                return;
            else if (spindexColorRightState.equals(targetColor))
                runSpindexToNextArtifact(1);
            else
                runSpindexToNextArtifact(2);
            updateSpinColorSensors();
        }
    }

    //0 is 0, 1 is negative, 2 is positive.
    public void runSpindexToNextArtifact(int direction) {
//        spindexMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

//        spindexMotor.setTargetPosition((encoderTicksForNextArtifact) + spindexMotor.getCurrentPosition());
        if (getColor(spindexColorBack).equals(GeneralConstants.artifactColors.EMPTY)) {
            double velocity = (direction == 0 ? 0 : direction == 1 ? -1 : 1) * spindexColorBack.getDistance(DistanceUnit.INCH) * 20;
            spindexMotor.setVelocity(velocity);
            if (!getColor(spindexColorLeft).equals(GeneralConstants.artifactColors.EMPTY) || !getColor(spindexColorRight).equals(GeneralConstants.artifactColors.EMPTY))
                return;
        }
    }

    public GeneralConstants.artifactColors getColor(ColorRangeSensor colorSensor) {
        float r = colorSensor.red();
        float g = colorSensor.green();
        float b = colorSensor.blue();


        if (g < 980 && g > 400 && r > 330 && b > 570 && colorSensor.getDistance(DistanceUnit.INCH) < 4)
            return GeneralConstants.artifactColors.PURPLE;
        else if (r < 330 && b > 740 && g > 980 && colorSensor.getDistance(DistanceUnit.INCH) < 4)
            return GeneralConstants.artifactColors.GREEN;
        else
            return GeneralConstants.artifactColors.EMPTY;
    }

    public String getColor(ColorRangeSensor colorRangeSensor, boolean irrelevant) {
        float r = colorRangeSensor.red();
        float g = colorRangeSensor.green();
        float b = colorRangeSensor.blue();

        return "R: " + r + " G: " + g + " B: " + b;
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
