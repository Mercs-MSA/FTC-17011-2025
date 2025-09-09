package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private CRServo intakeServo;
    private ColorSensor intakeColor;

    public Intake(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        intakeColor = hardwareMap.get(ColorSensor.class, "intakeColor");

        intakeServo.setPower(0);
    }

    // Control the continuous rotation servo
    public void setPower(double power) {
        intakeServo.setPower(power);
    }

    // Detects ball color: "Purple", "Green", or "Unknown"
    public String detectBallColor() {
        float r = intakeColor.red();
        float g = intakeColor.green();
        float b = intakeColor.blue();

        if (r > 100 && b > 100 && g < 80) {
            return "Purple";
        } else if (g > 120 && r < 100 && b < 100) {
            return "Green";
        } else {
            return "Unknown";
        }
    }
}
