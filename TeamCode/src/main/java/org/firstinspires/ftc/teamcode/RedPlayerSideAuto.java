package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static org.firstinspires.ftc.teamcode.Constants.Constants.onBlueAlliance;
import static org.firstinspires.ftc.teamcode.Constants.Constants.ranAuto;

@Autonomous(name = "Red Player Side Auto", group = "Competition")
public class RedPlayerSideAuto extends OpMode {

    private Follower follower;
    private SoftElectronics softElectronics;
    private Intake intake;
    private Shooter shooter;

    private FtcDashboard dash;
    private Telemetry telemetryA;

    private ElapsedTime shootTimer;

    private enum AutoState {
        START,
        PATH_CHAIN_RUNNING,
        SPINUP_AND_SHOOT,
        DONE
    }

    private AutoState state = AutoState.START;

    // --- Shooter control ---
    public static int shooterVelocity = 6000;   // ticks / sec, tune as needed

    // -----------------------------
    // PATH / POSE DEFINITIONS (RED)
    // -----------------------------

    // Start at the “hub” point, facing 90° (upfield)
    public static final Pose startPose = new Pose(
            95.8554216, 95.6385542, Math.toRadians(90)
    );

    // End of Path 1: same XY, rotated to 180°
    public static final Pose poseRotateEnd = new Pose(
            95.8554216, 95.6385542, Math.toRadians(180)
    );

    public static final Pose p2 = new Pose(109.0843370, 83.4939759, 0);
    public static final Pose p3 = new Pose(129.0361445, 83.4939759, 0);
    public static final Pose p4 = new Pose(95.8554216, 95.6385542, 0);
    public static final Pose p5 = new Pose(109.7349390, 59.4216867, 0);
    public static final Pose p6 = new Pose(128.6024090, 59.2048192, 0);
    public static final Pose p7 = new Pose(95.6385542, 95.8554216, 0);
    public static final Pose p8 = new Pose(110.8192771, 35.5662650, 0);
    public static final Pose p9 = new Pose(129.2530120, 35.3493975, 0);
    public static final Pose p10 = new Pose(95.8554216, 95.8554216, 0);
    public static final Pose p11 = new Pose(119.4939759, 95.6385542, 0);

    public static final Path path1 = new Path(new BezierLine(startPose, poseRotateEnd));
    public static final Path path2 = new Path(new BezierLine(poseRotateEnd, p2));
    public static final Path path3 = new Path(new BezierLine(p2, p3));
    public static final Path path4 = new Path(new BezierLine(p3, p4));
    public static final Path path5 = new Path(new BezierLine(p4, p5));
    public static final Path path6 = new Path(new BezierLine(p5, p6));
    public static final Path path7 = new Path(new BezierLine(p6, p7));
    public static final Path path8 = new Path(new BezierLine(p7, p8));
    public static final Path path9 = new Path(new BezierLine(p8, p9));
    public static final Path path10 = new Path(new BezierLine(p9, p10));
    public static final Path path11 = new Path(new BezierLine(p10, p11));

    public static final PathChain fullChain = new PathChain(
            path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11
    );

    static {
        // Path 1: explicit 90° -> 180° heading sweep, no translation
        path1.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180));

        // All others: tangential heading like in the composer
        path2.setTangentHeadingInterpolation();
        path3.setTangentHeadingInterpolation();
        path4.setTangentHeadingInterpolation();
        path5.setTangentHeadingInterpolation();
        path6.setTangentHeadingInterpolation();
        path7.setTangentHeadingInterpolation();
        path8.setTangentHeadingInterpolation();
        path9.setTangentHeadingInterpolation();
        path10.setTangentHeadingInterpolation();
        path11.setTangentHeadingInterpolation();
    }

    @Override
    public void init() {
        softElectronics = new SoftElectronics(hardwareMap, this.telemetry);
        dash = FtcDashboard.getInstance();
        telemetryA = new MultipleTelemetry(telemetry, dash.getTelemetry());

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shootTimer = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(1.0);

        onBlueAlliance = false;
        ranAuto = true;

        telemetryA.addLine("Red Mock Path Auto Init");
        telemetryA.update();
    }

    @Override
    public void start() {
        super.start();
        shooter.setMotorVelocity(0);
        intake.setPower(0);
        state = AutoState.START;
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case START:
                // Begin following the full chain & start intaking
                follower.followPath(fullChain);
                intake.setPower(1.0);
                state = AutoState.PATH_CHAIN_RUNNING;
                break;

            case PATH_CHAIN_RUNNING:
                if (!follower.isBusy()) {
                    // Finished the path, stop intake and start shooter spinup
                    intake.setPower(0.0);
                    shooter.setMotorVelocity(shooterVelocity);
                    shootTimer.reset();
                    state = AutoState.SPINUP_AND_SHOOT;
                }
                break;

            case SPINUP_AND_SHOOT:
                double v = shooter.getVelocity();

                // Simple spin-up check + timed feed
                if (v > shooterVelocity * 0.9) {
                    // Feed with intake for ~1.5 seconds once at speed
                    if (shootTimer.seconds() < 1.5) {
                        intake.setPower(1.0);
                    } else {
                        intake.setPower(0.0);
                        shooter.setMotorVelocity(0);
                        state = AutoState.DONE;
                    }
                }
                break;

            case DONE:
                shooter.setMotorVelocity(0);
                intake.setPower(0.0);
                break;
        }

        telemetryA.addData("State", state);
        telemetryA.addData("Pose X", follower.getPose().getX());
        telemetryA.addData("Pose Y", follower.getPose().getY());
        telemetryA.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetryA.addData("Shooter vel", shooter.getVelocity());
        telemetryA.addData("Timer", shootTimer.seconds());
        telemetryA.update();
    }
}