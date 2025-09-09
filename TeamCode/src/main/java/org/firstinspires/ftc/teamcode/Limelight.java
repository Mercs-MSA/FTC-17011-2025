package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp
public class Limelight extends OpMode {
    private Limelight3A limelight;
    private IMU imu1;
    private FtcDashboard dash;
    private Telemetry telemetryA;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu1 = hardwareMap.get(IMU.class, "imu");
        telemetry.setMsTransmissionInterval(11);
        dash = FtcDashboard.getInstance();
        telemetryA = new MultipleTelemetry(this.telemetry, dash.getTelemetry());
        limelight.pipelineSwitch(3);
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu1.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }
                Pose3D botpose = result.getBotpose_MT2();
                // Use botpose data
                telemetryA.addData("pitch", botpose.getOrientation().getPitch(AngleUnit.DEGREES));
                telemetryA.update();
            }
        }
    }
}