package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Teleop;

@Config
public class Spindex {
    public static DcMotorEx spindexMotor;
//    public ColorRangeSensor spindexColorBack; //Closest to wheel
    public ColorRangeSensor spindexColorRight; //Right of the wheel
//    public ColorRangeSensor spindexColorLeft; //Left of the wheel

    private DigitalChannel entrySensor;


    public static int currentSpindexPosition = 0;

    public static int spindexPositionFromAuto = 0;




    private GeneralConstants.colorSensorStates spindexColorBackState;
    private GeneralConstants.colorSensorStates spindexColorRightState;
    private GeneralConstants.colorSensorStates spindexColorLeftState;

    private static CRServo spindexTransferServo;
    private static int numOfArtifactsInRobot = 0;
    private int numSnapshot = 0;

    public static double spindexMotorVelocity = 600;

    private boolean alreadyChecked = false;


    public GeneralConstants.colorSensorStates targetColor = GeneralConstants.colorSensorStates.EMPTY;



    public Spindex(HardwareMap hardwareMap) {
        spindexMotor = hardwareMap.get(DcMotorEx.class, "spindexMotor");

        spindexTransferServo = hardwareMap.get(CRServo.class, "spindexTransferServo");
//        spindexColorBack = hardwareMap.get(ColorRangeSensor.class, "spindexColorB");
        spindexColorRight = hardwareMap.get(ColorRangeSensor.class, "spindexColorR");
//        spindexColorLeft = hardwareMap.get(ColorRangeSensor.class, "spindexColorL");

        entrySensor = hardwareMap.get(DigitalChannel.class, "entrySensor");
        entrySensor.setMode(DigitalChannel.Mode.INPUT);

        spindexColorBackState = GeneralConstants.colorSensorStates.EMPTY;
        spindexColorRightState = GeneralConstants.colorSensorStates.EMPTY;
        spindexColorLeftState = GeneralConstants.colorSensorStates.EMPTY;

        spindexMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        spindexMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentSpindexPosition = 0;
        spindexMotor.setTargetPosition(currentSpindexPosition);
        spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexMotor.setVelocity(spindexMotorVelocity);

        spindexTransferServo.setDirection(CRServo.Direction.REVERSE);
        spindexTransferServo.setPower(0);
    }


    public void changeCurrentPositionBy(int positionChange) {
        currentSpindexPosition += positionChange;
        spindexMotor.setTargetPosition(currentSpindexPosition);
        spindexMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spindexMotor.setVelocity(spindexMotorVelocity);
    }

    public void resetSpindexToZero() {
        currentSpindexPosition = 0;
        spindexMotor.setTargetPosition(currentSpindexPosition);
        spindexMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spindexMotor.setVelocity(800);
    }

    public void runSpindexToTransferThird() {
        if (currentSpindexPosition % Teleop.spindexThirdRevolution != 0 ) {
            changeCurrentPositionBy(Teleop.spindexThirdRevolution/2);
        }
    }


    public void stopSpindex() {
        spindexMotor.setVelocity(0);
    }

    public double getSpindexVelocity() {
        return spindexMotor.getVelocity();
    }



    public void runTransferWheel() {
        spindexTransferServo.setPower(1);
    }

    public void reverseTransfer() {
        spindexTransferServo.setPower(-1);
    }


    public void stopTransferWheel() {
        spindexTransferServo.setPower(0);
    }


    public void setSpindexColorTarget(GeneralConstants.colorSensorStates targetColor) {
        this.targetColor = targetColor;
        numSnapshot = numOfArtifactsInRobot;
        alreadyChecked = false;
    }

    public GeneralConstants.colorSensorStates getColor(ColorRangeSensor colorSensor) {
        float r = colorSensor.red();
        float g = colorSensor.green();
        float b = colorSensor.blue();

        if (r < 200 && g < 380 && b < 360) { //Green: 234, 428, 382 || Purple: 250, 410, 403
            return GeneralConstants.colorSensorStates.EMPTY;
        } else {
            return GeneralConstants.colorSensorStates.OCCUPIED;
        }

        /*
        if (colorSensor.equals(spindexColorBack)) {
//            if (g < 980 && g > 400 && r > 330 && b > 570 && colorSensor.getDistance(DistanceUnit.INCH) < 3)
//                return GeneralConstants.artifactColors.PURPLE;
//            else if (r < 330 && b > 740 && g > 980 && colorSensor.getDistance(DistanceUnit.INCH) < 3)
//                return GeneralConstants.artifactColors.GREEN;
//            else
//                return GeneralConstants.artifactColors.EMPTY;
            if (g > r && g > (b + 5) && colorSensor.getDistance(DistanceUnit.INCH) < 2.3)
                return GeneralConstants.artifactColors.GREEN;
            else if (colorSensor.getDistance(DistanceUnit.INCH) < 2.3)
                return GeneralConstants.artifactColors.PURPLE;
            else
                return GeneralConstants.artifactColors.EMPTY;
        } else {
//            if (g < 980 && g > 400 && r > 330 && b > 570 && colorSensor.getDistance(DistanceUnit.INCH) < 1.5)
//                return GeneralConstants.artifactColors.PURPLE;
//            else if (r < 330 && b > 740 && g > 980 && colorSensor.getDistance(DistanceUnit.INCH) < 1.5)
//                return GeneralConstants.artifactColors.GREEN;            else
//                return GeneralConstants.artifactColors.EMPTY;
            if (g > r && g > b && colorSensor.getDistance(DistanceUnit.INCH) < 1.5)
                return GeneralConstants.artifactColors.GREEN;
            else if (colorSensor.getDistance(DistanceUnit.INCH) < 1.5)
                return GeneralConstants.artifactColors.PURPLE;
            else
                return GeneralConstants.artifactColors.EMPTY;
        }
        */
    }

    public String getColor(ColorRangeSensor colorRangeSensor, boolean irrelevant) {
        float r = colorRangeSensor.red();
        float g = colorRangeSensor.green();
        float b = colorRangeSensor.blue();

        return "R: " + r + " G: " + g + " B: " + b;
    }


    public void updateSpinColorSensors() {
        numOfArtifactsInRobot = 0;
//        spindexColorBackState = getColor(spindexColorBack);
//        spindexColorLeftState = getColor(spindexColorLeft);
        spindexColorRightState = getColor(spindexColorRight);

        numOfArtifactsInRobot += (spindexColorBackState.equals(GeneralConstants.colorSensorStates.EMPTY)) ? 0 : 1;
        numOfArtifactsInRobot += (spindexColorLeftState.equals(GeneralConstants.colorSensorStates.EMPTY)) ? 0 : 1;
        numOfArtifactsInRobot += (spindexColorRightState.equals(GeneralConstants.colorSensorStates.EMPTY)) ? 0 : 1;
    }

    public int getNumOfArtifactsInRobot() {
        updateSpinColorSensors();
        return numOfArtifactsInRobot;
    }

    public boolean checkIfIntaked() {
        return entrySensor.getState();
    }
}
