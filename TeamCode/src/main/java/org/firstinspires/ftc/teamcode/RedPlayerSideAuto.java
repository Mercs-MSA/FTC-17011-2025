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

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class RedPlayerSideAuto extends OpMode {
    private Follower follower;
    private Telemetry telemetryA;
    private FtcDashboard dash;

    private double botHeading;

    private enum AUTO_STATES {
        PATH_ACTIVE,
        START,
        INACTIVE,
        SHOOT_STATE,
        PATH_TO_INTAKE1,
        INTAKE_STATE,
        PATH_TO_SHOOT2,
    }

    private AUTO_STATES currentState = AUTO_STATES.START;
    private AUTO_STATES nextState = AUTO_STATES.INACTIVE;

    /// ALL POINTS/PATHS HERE
    public static final Pose startPose = new Pose(0, 0, 0);
    public static final Pose examplePose = new Pose(0,0,0);
    public static final Pose shootPose = new Pose(0,0,0);
    public static final Pose intakePose1 = new Pose(0,0,0);

    public static final Path examplePath = new Path(new BezierLine(startPose, examplePose));
    public static final Path shootPath1 = new Path(new BezierLine(startPose, shootPose));
    public static final Path intakePath1 = new Path(new BezierLine(shootPose, intakePose1));
    public static final Path shootPath2 = new Path(new BezierLine(intakePose1, shootPose ));

    public static final double shootHeading = Math.toRadians(90);
    public static final double intakeHeading = Math.toRadians(90);
    @Override
    public void init() {
        telemetryA = new MultipleTelemetry(telemetry, dash.getTelemetry());
    }

    /// ALL FUNCTIONS HERE
    private void startState() {
        setupPath(shootPath1, shootHeading);
        //Sort Balls while Driving
        currentState = AUTO_STATES.PATH_ACTIVE;
        nextState = AUTO_STATES.SHOOT_STATE;
    }
    private void shootState() {
        //Shoots balls here
        currentState = AUTO_STATES.PATH_ACTIVE;
        nextState = AUTO_STATES.PATH_TO_INTAKE1;
    }
    private void pathIntake1() {
        setupPath(intakePath1, intakeHeading);
        currentState = AUTO_STATES.PATH_ACTIVE;
        nextState = AUTO_STATES.INTAKE_STATE;
    }
    private void intakeState() {
        //Intakes Balls here
        currentState = AUTO_STATES.PATH_ACTIVE;
        nextState = AUTO_STATES.PATH_TO_SHOOT2;
    }
    private void pathShoot2() {
        setupPath(shootPath2, shootHeading);
        //Sort Balls while moving
        currentState = AUTO_STATES.PATH_ACTIVE;
        nextState = AUTO_STATES.SHOOT_STATE;
    }

    private void pathActiveState() {
        if (!follower.isBusy()) {
            currentState = nextState;
        }
    }

    private void setupPath(Path pathToFollow, double endHeading) {
        double currentHeading = botHeading;
        follower.followPath(pathToFollow);
        pathToFollow.setLinearHeadingInterpolation(currentHeading, endHeading);
        botHeading = endHeading;
    }

    private void setupPathChain(PathChain pathChainToFollow, double endHeading) {
        double currentHeading = botHeading;
        follower.followPath(pathChainToFollow);
        botHeading = endHeading;
//        initalizePathHeadings();
    }

    private void inactiveState() {}

    private void stateMachine() {
        switch (currentState) {
            case START: startState(); break;
            case PATH_ACTIVE: pathActiveState(); break;
            case SHOOT_STATE: shootState(); break;
            case PATH_TO_INTAKE1: pathIntake1(); break;
            case INTAKE_STATE: intakeState(); break;
            case PATH_TO_SHOOT2: pathShoot2(); break;
            case INACTIVE: inactiveState(); break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        stateMachine();

        telemetryA.addData("current state", currentState);
        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("heading", follower.getPose().getHeading());
        telemetryA.update();
    }
}
