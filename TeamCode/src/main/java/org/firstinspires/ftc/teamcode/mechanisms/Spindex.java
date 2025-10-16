package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.Teleop.spinningToColor;

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
    public static double spindexVelocity = -800;
    public static double spindexThirdCount = 468;



    private GeneralConstants.artifactColors spindexColorBackState;
    private GeneralConstants.artifactColors spindexColorRightState;
    private GeneralConstants.artifactColors spindexColorLeftState;

    private static CRServo spindexTransferServo;
    private static Servo spindexGateServo;

    private static int numOfArtifactsInRobot = 0;
    private int numSnapshot = 0;

    public static double spindexGateOpenPosition = .15;
    public static double spindexGateClosedPosition = .28;

    private boolean alreadyChecked = false;

    public GeneralConstants.artifactColors targetColor = GeneralConstants.artifactColors.EMPTY;


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
        spindexMotor.setTargetPosition(0);

        spindexTransferServo.setPower(0);
        spindexGateServo.setPosition(spindexGateOpenPosition);
    }

    public void openSpindexGate() {
        spindexGateServo.setPosition(spindexGateOpenPosition);
    }

    public void closeSpindexGate () {
        spindexGateServo.setPosition(spindexGateClosedPosition);
    }

    public void runSpindex(boolean reverseTrue) {
        spindexMotor.setVelocity((reverseTrue ? -1 : 1) * spindexVelocity);
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

    public void setSpindexColorTarget(GeneralConstants.artifactColors targetColor) {
        this.targetColor = targetColor;
        numSnapshot = numOfArtifactsInRobot;
        alreadyChecked = false;
    }

    public void runSpindexToColor() {
        updateSpinColorSensors();
        if (numSnapshot == 0) {
            if (!((Math.abs(spindexMotor.getCurrentPosition()) / spindexThirdCount) < 3.05) && !((Math.abs(spindexMotor.getCurrentPosition()) / spindexThirdCount) > 2.95)) {
                spindexMotor.setVelocity(spindexVelocity);
                if (numOfArtifactsInRobot > 0)
                    numSnapshot = numOfArtifactsInRobot;
            } else {
                //STOP
                stopSpindex();
                spinningToColor = false;
                targetColor = GeneralConstants.artifactColors.EMPTY;
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
                    targetColor = GeneralConstants.artifactColors.EMPTY;
                } else {
                    spindexMotor.setVelocity(spindexVelocity);
                }
//            }
        }
    }

    //0 is 0, 1 is negative, 2 is positive.
    public void runSpindexToNextArtifact(int direction) {
//        spindexMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

//        spindexMotor.setTargetPosition((encoderTicksForNextArtifact) + spindexMotor.getCurrentPosition());
        if (getColor(spindexColorBack).equals(GeneralConstants.artifactColors.EMPTY)) {
//            double velocity = (direction == 0 ? 0 : direction == 1 ? -1 : 1) * spindexColorBack.getDistance(DistanceUnit.INCH) * 20;
//            double velocity = spindexColorBack.getDistance(DistanceUnit.INCH) * 20;
            spindexMotor.setVelocity(spindexVelocity);
            if (!getColor(spindexColorLeft).equals(GeneralConstants.artifactColors.EMPTY) || !getColor(spindexColorRight).equals(GeneralConstants.artifactColors.EMPTY))
                return;
        }
    }

    public GeneralConstants.artifactColors getColor(ColorRangeSensor colorSensor) {
        float r = colorSensor.red();
        float g = colorSensor.green();
        float b = colorSensor.blue();

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

        numOfArtifactsInRobot += (spindexColorBackState.equals(GeneralConstants.artifactColors.EMPTY)) ? 0 : 1;
        numOfArtifactsInRobot += (spindexColorLeftState.equals(GeneralConstants.artifactColors.EMPTY)) ? 0 : 1;
        numOfArtifactsInRobot += (spindexColorRightState.equals(GeneralConstants.artifactColors.EMPTY)) ? 0 : 1;
    }

    public int getNumOfArtifactsInRobot() {
        updateSpinColorSensors();
        return numOfArtifactsInRobot;
    }
}
