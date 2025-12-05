package org.firstinspires.ftc.teamcode.Autos;

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

import static org.firstinspires.ftc.teamcode.Constants.Constants.currentTheta;
import static org.firstinspires.ftc.teamcode.Constants.Constants.currentX;
import static org.firstinspires.ftc.teamcode.Constants.Constants.currentY;
import static org.firstinspires.ftc.teamcode.Constants.Constants.onBlueAlliance;
import static org.firstinspires.ftc.teamcode.Constants.Constants.ranAuto;

@Autonomous(name = "Red Player Side Auto", group = "Competition")
public class RedPlayerSideAuto extends OpMode {

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

        currentX = 0;
        currentY = 0;
        currentTheta = 0;

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

        switch (currentState) {
            case START:
                follower.followPath(startPath);
                shooter.setTurretTarget(-23.8);
                currentState = AutoState.PATH_ACTIVE;
                nextState = AutoState.SHOOT;
                previousState = AutoState.START;
                break;

            case PATH_ACTIVE:
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = nextState;
                }
                break;

            case SHOOT:
                shootingMachine();
                if (shootingState.equals(Teleop.SHOOTING_STATE.INACTIVE)) {
                    intake.setPower(0);
                    shootingState = Teleop.SHOOTING_STATE.START;
                } else if (shootingState.equals(Teleop.SHOOTING_STATE.END)) {
                    if (previousState.equals(AutoState.START)) {
                        currentState = AutoState.PATH_INTAKE_1;
                    } else if (previousState.equals(AutoState.PATH_INTAKE_1)) {
                        currentState = AutoState.PATH_INTAKE_2;
                    } else if (previousState.equals(AutoState.PATH_INTAKE_2)) {
                        currentState = AutoState.END_PATH;
                    }
                }
                break;

            case PATH_INTAKE_1:
                intake.setPower(1);
                follower.setMaxPower(.4);
                follower.followPath(intakeChain1);
                currentState = AutoState.PATH_ACTIVE;
                nextState = AutoState.SHOOT;
                previousState = AutoState.PATH_INTAKE_1;
                break;

            case PATH_INTAKE_2:
                intake.setPower(1);
                follower.setMaxPower(.4);
                follower.followPath(intakeChain2);
                currentState = AutoState.PATH_ACTIVE;
                nextState = AutoState.SHOOT;
                previousState = AutoState.PATH_INTAKE_2;
                break;

            case END_PATH:
                follower.followPath(exitPath);
                shooter.setTurretTarget(0);
                currentState = AutoState.PATH_ACTIVE;
                nextState = AutoState.DONE;
                break;

            case DONE:
                shooter.setMotorVelocity(0);
                intake.setPower(0.0);
                break;
        }

        currentX = follower.getPose().getX();
        currentY = follower.getPose().getY();
        currentTheta = follower.getPose().getHeading();

        telemetryA.addData("State", currentState);
        telemetryA.addData("Pose X", follower.getPose().getX());
        telemetryA.addData("Pose Y", follower.getPose().getY());
        telemetryA.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetryA.addData("Shooter vel", shooter.getVelocity());
        telemetryA.addData("Timer", shootTimer.seconds());
        telemetryA.update();
    }

    private boolean desiredVelocityReached() {
        if (shooter.getVelocity() > shooterVelocity * .9)
            return true;
        return false;
    }

    int shotCount = 0;
    private void shootingMachine() {
        switch (shootingState) {
            case START:
                shooter.setMotorVelocity(shooterVelocity);
                shootingState = Teleop.SHOOTING_STATE.SPIN_UP;
                break;
            case SPIN_UP:
                transfer.closeTransferGate();
                if (desiredVelocityReached()) {
                    shootingState = Teleop.SHOOTING_STATE.SHOOT;
                }
                break;
            case SHOOT:
                transfer.openTransferGate();
                transfer.setPower(.87);
                if (shotCount < 2 && !desiredVelocityReached()) {
                    shotCount++;
                    shootingState = Teleop.SHOOTING_STATE.SPIN_UP;
                } else if (shotCount >= 2) {
                    shotCount = 0;
                    shootingState = Teleop.SHOOTING_STATE.END;
                }
                break;
            case END:
                shooter.setMotorVelocity(0);
                transfer.closeTransferGate();
                transfer.setPower(0);
                shootingState = Teleop.SHOOTING_STATE.INACTIVE;
                break;
            case INACTIVE:
                transfer.closeTransferGate();
                break;
        }
    }
}