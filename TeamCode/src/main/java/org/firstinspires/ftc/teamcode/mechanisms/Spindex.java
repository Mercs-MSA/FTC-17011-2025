package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class Spindex {

    private DcMotor spindexMotor;
    private ColorSensor spindexColor;
    private Servo spindexServo;

    public Spindex(HardwareMap hardwareMap) {
        spindexMotor = hardwareMap.get(DcMotor.class, "spindexMotor");
        spindexColor = hardwareMap.get(ColorSensor.class, "spindexColor");
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
    public String detectBallColor() {
        float r = spindexColor.red();
        float g = spindexColor.green();
        float b = spindexColor.blue();

        if (r > 100 && b > 100 && g < 80) {
            return "Purple";
        } else if (g > 120 && r < 100 && b < 100) {
            return "Green";
        } else {
            return "Unknown";
        }
    }
}
