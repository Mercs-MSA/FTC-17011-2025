package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.localization.localizers.OTOSLocalizer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Configurable
public class AutoPractice extends OpMode {
    private Follower follower;
    private Telemetry telemetryA;
    private FtcDashboard dash;

    private double botHeading;

    private enum AUTO_STATES {
        PATH_ACTIVE,
        START,
        INACTIVE
    }

    private AUTO_STATES currentState = AUTO_STATES.START;
    private AUTO_STATES nextState = AUTO_STATES.INACTIVE;

    /// ALL POINTS/PATHS HERE
    public static final Pose startPose = new Pose(0, 0, 0);
    public static final Pose examplePose = new Pose(62.7,0,0);

    public static final Path examplePath = new Path(new BezierLine(startPose, examplePose));


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        dash = FtcDashboard.getInstance();
        telemetryA = new MultipleTelemetry(telemetry, dash.getTelemetry());
    }

    /// ALL FUNCTIONS HERE
    private void startState() {
        setupPath(examplePath, 0);
        currentState = AUTO_STATES.PATH_ACTIVE;
        nextState = AUTO_STATES.INACTIVE;
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
