package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.Teleop.spinningToColor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;

@Config
public class Spindex {
    private static DcMotorEx spindexMotor;
    public ColorRangeSensor spindexColorBack; //Closest to wheel
    public ColorRangeSensor spindexColorRight; //Right of the wheel
    public ColorRangeSensor spindexColorLeft; //Left of the wheel
    public static int spindexFullRevolution = 300; //Amount of encoder positions for one full revolution of spindex
    public static int spindexThirdRevolution = (int)(spindexFullRevolution/3.0); //Amount of encoder positions for one full revolution of spindex
    private int spindexOffset = 0;




    private GeneralConstants.colorSensorStates spindexColorBackState;
    private GeneralConstants.colorSensorStates spindexColorRightState;
    private GeneralConstants.colorSensorStates spindexColorLeftState;

    private static CRServo spindexTransferServo;
    private static int numOfArtifactsInRobot = 0;
    private int numSnapshot = 0;

    public static double spindexMotorVelocity = 400;

    public static double spindexGateOpenPosition = .15;
    public static double spindexGateClosedPosition = .285;

    private boolean alreadyChecked = false;

    public GeneralConstants.colorSensorStates targetColor = GeneralConstants.colorSensorStates.EMPTY;


    //    public enum SPIN_STATES {
//        INACTIVE,
//        ACTIVE_GREEN,
//        ACTIVE_PURPLE
//    }
//
//    private SPIN_STATES currentSpinState = SPIN_STATES.INACTIVE;


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
        spindexMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        spindexMotor.setVelocity(0);
        spindexMotor.setTargetPosition(0);

        spindexTransferServo.setPower(0);
    }

    public void runSpindex() {
        spindexMotor.setVelocity(spindexMotorVelocity);
    }

    public void runSpindexToTransfer() {
        spindexMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        int offset = spindexMotor.getCurrentPosition() % 300;
        while (offset < 0) {
            offset += 300;
        }

        spindexMotor.setTargetPosition(spindexMotor.getCurrentPosition() + offset);

        spindexMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runSpindexToEmpty() {
        spindexMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        spindexMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }




    public void stopSpindex() {
        spindexMotor.setVelocity(0);
    }

    public double getSpindexVelocity() {
        return spindexMotor.getVelocity();
    }

    public void resetSpindexEncodersByOffset() {
        spindexOffset = spindexMotor.getCurrentPosition() - spindexFullRevolution;
    }

    public int getSpindexPosition() {
        return spindexMotor.getCurrentPosition() - spindexOffset;
    }

    public void runTransferWheel() {
        spindexTransferServo.setPower(1);
    }

    public void stopTransferWheel() {
        spindexTransferServo.setPower(0);
    }


//    public void runSpindexToColor(GeneralConstants.artifactColors targetColor) {
//        updateSpinColorSensors();
//        if (targetColor.equals(GeneralConstants.artifactColors.EMPTY)) {
//            throw new IllegalArgumentException("Target color cannot be empty");
//        } else if (!spindexColorRightState.equals(targetColor) && !spindexColorLeftState.equals(targetColor) && !spindexColorBackState.equals(targetColor)) {
//            return;
//        } else {
//            if (spindexColorBackState.equals(targetColor))
//                return;
//            else if (spindexColorRightState.equals(targetColor))
//                runSpindexToNextArtifact(1);
//            else
//                runSpindexToNextArtifact(2);
//            updateSpinColorSensors();
//        }
//    }

    public void setSpindexColorTarget(GeneralConstants.colorSensorStates targetColor) {
        this.targetColor = targetColor;
        numSnapshot = numOfArtifactsInRobot;
        alreadyChecked = false;
    }

    public void runSpindexToColor() {
        updateSpinColorSensors();
        if (numSnapshot == 0) {
            if (!((Math.abs(spindexMotor.getCurrentPosition()) / spindexFullRevolution) < 3.05) && !((Math.abs(spindexMotor.getCurrentPosition()) / spindexFullRevolution) > 2.95)) {
                runSpindex();
                if (numOfArtifactsInRobot > 0)
                    numSnapshot = numOfArtifactsInRobot;
            } else {
                //STOP
                stopSpindex();
                spinningToColor = false;
                targetColor = GeneralConstants.colorSensorStates.EMPTY;
            }
//        } else if (!spindexColorRightState.equals(targetColor) && !spindexColorLeftState.equals(targetColor) && !spindexColorBackState.equals(targetColor)) {
//            stopSpindex();
//            spinningToColor = false;
//            targetColor = GeneralConstants.artifactColors.EMPTY;
//        } else {
//            if (spindexColorBackState.equals(targetColor)) {
//                stopSpindex();
//                spinningToColor = false;
//                targetColor = GeneralConstants.artifactColors.EMPTY;
//            }
//            else {
//                spindexMotor.setVelocity(spindexVelocity);
//            }
//        }

        } else {
//            if (!spindexColorRightState.equals(targetColor) && !spindexColorLeftState.equals(targetColor) && !spindexColorBackState.equals(targetColor) && !alreadyChecked) {
//                stopSpindex();
//                spinningToColor = false;
//                targetColor = GeneralConstants.artifactColors.EMPTY;
//            } else {
//                alreadyChecked = true;
                if (spindexColorBackState.equals(targetColor)) {
                    stopSpindex();
                    spinningToColor = false;
                    targetColor = GeneralConstants.colorSensorStates.EMPTY;
                } else {
                    spindexMotor.setVelocity(spindexMotorVelocity);
                }
//            }
        }
    }

    //0 is 0, 1 is negative, 2 is positive.
    public void runSpindexToNextArtifact(int direction) {
        if (getColor(spindexColorBack).equals(GeneralConstants.colorSensorStates.EMPTY)) {
            spindexMotor.setVelocity(spindexMotorVelocity);
        }
    }

    public GeneralConstants.colorSensorStates getColor(ColorRangeSensor colorSensor) {
        float r = colorSensor.red();
        float g = colorSensor.green();
        float b = colorSensor.blue();

        if (r < 185) {
            return GeneralConstants.colorSensorStates.OCCUPIED;
        } else {
            return GeneralConstants.colorSensorStates.EMPTY;
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
        spindexColorBackState = getColor(spindexColorBack);
        spindexColorLeftState = getColor(spindexColorLeft);
        spindexColorRightState = getColor(spindexColorRight);

        numOfArtifactsInRobot += (spindexColorBackState.equals(GeneralConstants.colorSensorStates.EMPTY)) ? 0 : 1;
        numOfArtifactsInRobot += (spindexColorLeftState.equals(GeneralConstants.colorSensorStates.EMPTY)) ? 0 : 1;
        numOfArtifactsInRobot += (spindexColorRightState.equals(GeneralConstants.colorSensorStates.EMPTY)) ? 0 : 1;
    }

    public int getNumOfArtifactsInRobot() {
        updateSpinColorSensors();
        return numOfArtifactsInRobot;
    }
}
