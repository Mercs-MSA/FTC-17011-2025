package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class TeleopTemplate extends OpMode {
    //Variables
    public enum TeleStates {
        START_STATE
    }

    private TeleStates currentState = TeleStates.START_STATE;

    //Motors
    private DcMotor leftFront;

    //Servos


    //Misc
    FtcDashboard dash = null;
    Telemetry telemetryA = null;

    //All Functions
    public void configureMotors() {
        //Initialization here
    }

    public void liftArm() {
        if (gamepad1.a) {
            //Code here
        }
    }

    @Override
    public void init() {
        configureMotors();
        dash = FtcDashboard.getInstance();
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
    }

    //State Machine
    @Override
    public void loop() {
        liftArm(); //Function in loop
    }
}
