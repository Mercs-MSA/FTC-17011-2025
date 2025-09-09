package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SoftElectronics {
    private FtcDashboard dash;
    private Telemetry telemetryA;

    public SoftElectronics(Telemetry opModeTelemetry) {
        dash = FtcDashboard.getInstance();
        telemetryA = new MultipleTelemetry(opModeTelemetry, dash.getTelemetry());
        telemetryA.update();
    }

    public Telemetry getTelemetry() {
        return telemetryA;
    }

    public FtcDashboard getDashboard() {
        return dash;
    }
}
