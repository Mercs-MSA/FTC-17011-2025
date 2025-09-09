package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="Motor Test", group="Tests")
public class MotorTest extends OpMode {

    // Motors & Servos
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterMotor, spindexMotor;
    private CRServo intakeServo;

    // List of all motors/servos
    private Object[] motorList;
    private int currentIndex = 0;

    // Button debounce
    private boolean aPressedLastLoop = false;

    @Override
    public void init() {
        // Initialize motors/servos
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        spindexMotor = hardwareMap.get(DcMotor.class, "spindexMotor");
        intakeServo  = hardwareMap.get(CRServo.class, "intakeServo");

        // Add them to a list in order
        motorList = new Object[] {
                frontLeft, frontRight, backLeft, backRight,
                shooterMotor, spindexMotor, intakeServo
        };

        telemetry.addData("Status", "Initialized - Press A to cycle motors");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ------------------------
        // Cycle motor with A
        // ------------------------
        if (gamepad1.a && !aPressedLastLoop) {
            // Stop previous motor
            stopAllMotors();

            // Move to next motor in list
            currentIndex++;
            if (currentIndex >= motorList.length) currentIndex = 0;
        }
        aPressedLastLoop = gamepad1.a;

        // ------------------------
        // Run current motor based on triggers
        // ------------------------
        Object currentMotor = motorList[currentIndex];
        double power = 0;

        if (gamepad1.right_trigger > 0.1) {
            power = gamepad1.right_trigger; // forward
        } else if (gamepad1.left_trigger > 0.1) {
            power = -gamepad1.left_trigger; // backward
        }

        if (currentMotor instanceof DcMotor) {
            ((DcMotor) currentMotor).setPower(power);
            telemetry.addData("Testing Motor", ((DcMotor) currentMotor).getDeviceName());
        } else if (currentMotor instanceof CRServo) {
            ((CRServo) currentMotor).setPower(power);
            telemetry.addData("Testing Servo", "Intake Servo");
        }

        telemetry.addData("Motor Index", currentIndex);
        telemetry.addData("Power", power);
        telemetry.update();
    }

    private void stopAllMotors() {
        for (Object motor : motorList) {
            if (motor instanceof DcMotor) {
                ((DcMotor) motor).setPower(0);
            } else if (motor instanceof CRServo) {
                ((CRServo) motor).setPower(0);
            }
        }
    }
}
