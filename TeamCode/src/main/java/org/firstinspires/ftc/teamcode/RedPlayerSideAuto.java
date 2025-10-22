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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.mechanisms.Drivebase;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Spindex;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class RedPlayerSideAuto extends OpMode {
    private Follower follower;
    private SoftElectronics softElectronics;
    private Spindex spindex;
    private Intake intake;
    private Shooter shooter;
    private ElapsedTime shootTimer;

    private Telemetry telemetryA;
    private FtcDashboard dash;

    private double botHeading;

    private enum AUTO_STATES {
        PATH_ACTIVE,
        START,
        INACTIVE,
        SHOOT_STATE,
        SHOOT_STATE_TWO,
        PATH_TO_INTAKE1,
        INTAKE_STATE,
        PATH_TO_SHOOT2,
    }

    private enum SHOOTER_STATE {REMOVE_USER_CONTROL, RUN_SHOOTER_MOTOR_STATE, RUN_TRANSFER_STATE, RUN_SPINDEX_STATE, INACTIVE_STATE}
    private static SHOOTER_STATE singleShotState = SHOOTER_STATE.INACTIVE_STATE;
    public static int shooterDesiredVelocity = 1800;
    private int timesShot = 0;


    private AUTO_STATES currentState = AUTO_STATES.START;
    private AUTO_STATES nextState = AUTO_STATES.INACTIVE;
    private AUTO_STATES previousState = AUTO_STATES.INACTIVE;

    /// ALL POINTS/PATHS HERE
    public static final Pose startPose = new Pose(0, 0, 0);
    public static final Pose shootPose = new Pose(66.67,0,Math.toRadians(-45));
    public static final Pose intakePose1 = new Pose(70.298,-47.134,Math.toRadians(-90));

    public static final Path shootPath1 = new Path(new BezierLine(startPose, shootPose));
    public static final Path intakePath1 = new Path(new BezierLine(shootPose, intakePose1));
    public static final Path shootPath2 = new Path(new BezierLine(intakePose1, shootPose ));

    @Override
    public void init() {
        softElectronics = new SoftElectronics(hardwareMap, this.telemetry);

        spindex = new Spindex(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shootTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(1);

        dash = FtcDashboard.getInstance();
        telemetryA = new MultipleTelemetry(telemetry, dash.getTelemetry());
        Teleop.onBlueAlliance = false;
        Teleop.startingOrientation = Teleop.STARTING_ORIENTATION.GOAL_SIDE;
    }

    @Override
    public void start() {
        super.start();
        shooter.setShooterPower(1);
        shooter.setMotorVelocity(0);
        spindex.initSpindex();
    }

    /// ALL FUNCTIONS HERE
    private void startState() {
        setupPath(shootPath1, shootPose.getHeading());
        currentState = AUTO_STATES.PATH_ACTIVE;
        nextState = AUTO_STATES.SHOOT_STATE;
    }
    private void shootState() {
        intake.setPower(1);
        shooter.setMotorVelocity(shooterDesiredVelocity);
        if (shooter.getRightVelocity() > shooterDesiredVelocity * .8) {
            spindex.runSpindexToNextArtifact(2);
            if (!spindex.getColor(spindex.spindexColorBack).equals(GeneralConstants.artifactColors.EMPTY) && shooter.getRightVelocity() > shooterDesiredVelocity * .95) {
                currentState = AUTO_STATES.SHOOT_STATE_TWO;
                shootTimer.reset();
            }
        }
    }

    private void shootStatePart2() {
        intake.setPower(1);
        spindex.stopSpindex();
        spindex.runTransferWheel();
        if (singleShotState.equals(SHOOTER_STATE.INACTIVE_STATE) && shootTimer.time() > 3.76) {
            timesShot += 1;
            shootTimer.reset();
            if (timesShot < 3)
                currentState = AUTO_STATES.SHOOT_STATE;
            else {
                timesShot = 0;
                previousState = AUTO_STATES.SHOOT_STATE_TWO;
                currentState = AUTO_STATES.PATH_TO_INTAKE1;
            }
        }
    }
    private void pathIntake1() {
        setupPath(intakePath1, intakePose1.getHeading());
        follower.setMaxPower(.5);
        shooter.stop();
        currentState = AUTO_STATES.PATH_ACTIVE;
        nextState = AUTO_STATES.INTAKE_STATE;
    }
    private void intakeState() {
        //Intakes Balls here
        currentState = AUTO_STATES.PATH_ACTIVE;
        nextState = AUTO_STATES.PATH_TO_SHOOT2;
    }
    private void pathShoot2() {
        setupPath(shootPath2, shootPose.getHeading());
        //Sort Balls while moving
        currentState = AUTO_STATES.PATH_ACTIVE;
        nextState = AUTO_STATES.SHOOT_STATE;
    }

    private void pathActiveState() {
        if (!follower.isBusy()) {
            currentState = nextState;
        }
        if (previousState.equals(AUTO_STATES.SHOOT_STATE_TWO)) {
            intake.setPower(1);
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

    private void inactiveState() {
        shooter.stop();
        intake.setPower(0);
        spindex.stopSpindex();
        spindex.stopTransferWheel();
    }

    private void stateMachine() {
        switch (currentState) {
            case START: startState(); break;
            case PATH_ACTIVE: pathActiveState(); break;
            case SHOOT_STATE: shootState(); break;
            case SHOOT_STATE_TWO: shootStatePart2(); break;
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
        telemetryA.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryA.addData("shot timer:", shootTimer.time());
        telemetryA.update();
    }



    private void updateSingleShotStateMachine() {
        switch (singleShotState) {
            case RUN_SHOOTER_MOTOR_STATE:
                intake.setPower(1);
                shooter.setMotorVelocity(shooterDesiredVelocity);
                if (shooter.getRightVelocity() > shooterDesiredVelocity * .8) {
                    singleShotState = SHOOTER_STATE.RUN_SPINDEX_STATE;
                }
                break;
            case RUN_SPINDEX_STATE:
                intake.setPower(1);
                spindex.runSpindexToNextArtifact(2);
                spindex.stopTransferWheel();
                if (!spindex.getColor(spindex.spindexColorBack).equals(GeneralConstants.artifactColors.EMPTY) && shooter.getRightVelocity() > shooterDesiredVelocity * .95)
                    singleShotState = SHOOTER_STATE.RUN_TRANSFER_STATE;
                break;
            case RUN_TRANSFER_STATE:
                intake.setPower(1);
                spindex.stopSpindex();
                spindex.runTransferWheel();
                break;
            case INACTIVE_STATE:
                shooter.stop();
                spindex.stopTransferWheel();
                break;
        }
    }
}
