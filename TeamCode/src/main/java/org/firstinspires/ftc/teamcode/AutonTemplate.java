package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous
public class AutonTemplate extends OpMode {
    //Variables
    public enum AutoStates {
        START_STATE
    }

    private AutoStates currentState = AutoStates.START_STATE;

    //Motors


    //Servos


    //Misc
    FtcDashboard dash = null;
    Telemetry telemetryA = null;

    //All Functions
    public void configureMotors() {

    }

    @Override
    public void init() {
        configureMotors();
        dash = FtcDashboard.getInstance();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
    }


    //State Machine
    @Override
    public void loop() {

    }
}
