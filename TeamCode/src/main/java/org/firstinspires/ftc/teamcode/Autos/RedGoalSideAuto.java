package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Constants.Constants.onBlueAlliance;
import static org.firstinspires.ftc.teamcode.Constants.Constants.ranAuto;

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
import org.firstinspires.ftc.teamcode.SoftElectronics;
import org.firstinspires.ftc.teamcode.Teleop;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Goal Side Auto", group = "Competition")
public class RedGoalSideAuto extends OpMode {

    private Follower follower;
    private SoftElectronics softElectronics;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;

    private FtcDashboard dash;
    private Telemetry telemetryA;

    private ElapsedTime shootTimer;

    public static Teleop.SHOOTING_STATE shootingState = Teleop.SHOOTING_STATE.INACTIVE;
    private enum AutoState {
        START,
        PATH_ACTIVE,
        SHOOT,
        PATH_INTAKE_1,
        //        PATH_SHOOT_1,
        PATH_INTAKE_2,
        //        PATH_SHOOT_2,
        PATH_INTAKE_3,
        HOLD_GATE_1,
        PATH_INTAKE_4,
        HOLD_GATE_2,
        END_PATH,
        DONE,
        INACTIVE
    }

    private AutoState currentState = AutoState.INACTIVE;
    private AutoState nextState = AutoState.INACTIVE;
    private AutoState previousState = AutoState.INACTIVE;

    public static int shooterVelocity = 1925;

    // -----------------------------
    // PATH / POSE DEFINITIONS (BLUE)
    // Same XY, mirrored headings
    // -----------------------------

    public static final Pose startPose = new Pose(-61.3235, -15.1146, Math.toRadians(-180));

    public static final Pose shootPose = new Pose(-55.06379, -15.07, Math.toRadians(-180));
    public static final Pose readyIntakePose1 = new Pose(-34.41036, -28.7874, Math.toRadians(-90));
    public static final Pose intakeEndPose1 = new Pose(-35.67191, -58.8364, Math.toRadians(-90));
    public static final Pose readyIntakePose2 = new Pose(-45, -59.377, Math.toRadians(-135));
    public static final Pose intakeEndPose2 = new Pose(-60.8429, -60.11, Math.toRadians(-165));
//    public static final Pose intakeFromTunnel1 = new Pose(0,0, Math.toRadians(-45));
//    public static final Pose intakeFromTunnel2 = new Pose(0,0, Math.toRadians(-45));

    public static final Pose poseRotateEnd = new Pose(-46, -22, 0);


    public static final Path startPath = new Path(new BezierLine(startPose, shootPose));
    public static final Path readyIntakePath1 = new Path(new BezierLine(shootPose, readyIntakePose1));
    public static final Path endIntakePath1 = new Path(new BezierLine(readyIntakePose1, intakeEndPose1));
    public static final Path backToShoot1 = new Path(new BezierLine(intakeEndPose1, shootPose));
    public static final Path readyIntakePath2 = new Path(new BezierLine(shootPose, readyIntakePose2));
    public static final Path endIntakePath2 = new Path(new BezierLine(readyIntakePose2, intakeEndPose2));
    public static final Path backToShoot2 = new Path(new BezierLine(intakeEndPose2, shootPose));
    //    public static final Path toTunnel1 = new Path(new BezierLine(shootPose, intakeFromTunnel1));
//    public static final Path backToShoot3 = new Path(new BezierLine(intakeFromTunnel1, shootPose));
//    public static final Path toTunnel2 = new Path(new BezierLine(shootPose, intakeFromTunnel2));
//    public static final Path backToShoot4 = new Path(new BezierLine(intakeFromTunnel2, shootPose));
    public static final Path exitPath = new Path(new BezierLine(shootPose, poseRotateEnd));


    public static final PathChain intakeChain1 = new PathChain(readyIntakePath1, endIntakePath1, backToShoot1);
    public static final PathChain intakeChain2 = new PathChain(readyIntakePath2, endIntakePath2, backToShoot2);

    static {
        startPath.setTangentHeadingInterpolation();
        readyIntakePath1.setTangentHeadingInterpolation();
        endIntakePath1.setTangentHeadingInterpolation();
        backToShoot1.setTangentHeadingInterpolation();
        readyIntakePath2.setTangentHeadingInterpolation();
        endIntakePath2.setTangentHeadingInterpolation();
//        toTunnel1.setTangentHeadingInterpolation();
//        backToShoot3.setTangentHeadingInterpolation();
//        toTunnel2.setTangentHeadingInterpolation();
//        backToShoot4.setTangentHeadingInterpolation();
        exitPath.setTangentHeadingInterpolation();
    }

    @Override
    public void init() {
        softElectronics = new SoftElectronics(hardwareMap, this.telemetry);
        dash = FtcDashboard.getInstance();
        telemetryA = new MultipleTelemetry(telemetry, dash.getTelemetry());

        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shootTimer = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(-61.3235, -15.1146, Math.toRadians(-180)));

        follower.setMaxPower(.8);

        onBlueAlliance = false;
        ranAuto = true;

        telemetryA.addLine("Red Mock Path Auto Init");
        telemetryA.addData("Start Pose: ", follower.getPose());
        telemetryA.addData("Heading: ", Math.toDegrees(follower.getHeading()));
        telemetryA.update();
    }

    @Override
    public void start() {
        super.start();
        follower.setPose(startPose);
        shooter.setMotorVelocity(0);
        intake.setPower(0);
        currentState = AutoState.START;
        previousState = AutoState.INACTIVE;
        nextState = AutoState.INACTIVE;
    }

    @Override
    public void loop() {
        follower.update();
    }
}
