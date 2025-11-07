package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.Constants.onBlueAlliance;
import static org.firstinspires.ftc.teamcode.Constants.Constants.ranAuto;
import static org.firstinspires.ftc.teamcode.Teleop.spindexThirdRevolution;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Spindex;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;


@Autonomous
@Config
public class RedLongRangeAuto extends OpMode {
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
        WAIT,
        INACTIVE,
        SHOOT_STATE,
        SHOOT_STATE_TWO,
        PATH_TO_INTAKE1,
        INTAKE_STATE,
        PATH_TO_SHOOT2,
        LEAVE,
        END
    }

    private static Teleop.SHOOTER_STATE rapidFireState = Teleop.SHOOTER_STATE.INACTIVE_STATE;
    public static int shooterDesiredVelocity = 1625;
    private int timesShot = 0;
    private int waitTime = 6;

    public static int spinAmount = spindexThirdRevolution/3;



    private AUTO_STATES currentState = AUTO_STATES.START;
    private AUTO_STATES nextState = AUTO_STATES.INACTIVE;
    private AUTO_STATES previousState = AUTO_STATES.INACTIVE;

    /// ALL POINTS/PATHS HERE
    public static final Pose startPose = new Pose(0, 0, 0);
    public static final Pose shootPose = new Pose(4, 0,Math.toRadians(-20.3));
    public static final Pose intakePose1 = new Pose(10.8733,-9.81608,Math.toRadians(-90));
    public static final Pose intakePose2 = new Pose(0, 0, Math.toRadians(-90));
    public static final Pose leavePose = new Pose(8, -8, Math.toRadians(-90));


    public static final Path shootPath1 = new Path(new BezierLine(startPose, shootPose));
    public static final Path intakePath1 = new Path(new BezierLine(shootPose, intakePose1));
    public static final Path shootPath2 = new Path(new BezierLine(intakePose1, shootPose));
    public static final Path intakePath2 = new Path(new BezierLine(shootPose, intakePose2));
    public static final Path leavePath = new Path(new BezierLine(shootPose, leavePose));


    @Override
    public void init() {
        softElectronics = new SoftElectronics(hardwareMap, this.telemetry);

        spindex = new Spindex(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shootTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        rapidFireState = Teleop.SHOOTER_STATE.INACTIVE_STATE;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(1);

        dash = FtcDashboard.getInstance();
        telemetryA = new MultipleTelemetry(telemetry, dash.getTelemetry());
        onBlueAlliance = false;
        ranAuto = true;
        Teleop.startingOrientation = Teleop.STARTING_ORIENTATION.GOAL_SIDE;
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpadUpWasPressed()) {
            waitTime++;
        } else if (gamepad1.dpadDownWasPressed()) {
            if (waitTime < 0) {
                waitTime = 0;
            } else {
                waitTime--;
            }
        }

        telemetryA.addLine("To increase wait time by 1 press dpad up. Press dpad down to decrease by 1.");
        telemetryA.addData("Current wait time (in seconds):", waitTime);
    }

    @Override
    public void start() {
        super.start();
        shooter.setShooterPower(1);
        shooter.setMotorVelocity(0);
    }

    /// ALL FUNCTIONS HERE
    private void startState() {
        setupPath(shootPath1, shootPose.getHeading());
        currentState = AUTO_STATES.PATH_ACTIVE;
        nextState = AUTO_STATES.WAIT;
    }


    private void waitState() {
        if (shootTimer.time() > waitTime) {
            currentState = AUTO_STATES.SHOOT_STATE;
        }
        previousState = AUTO_STATES.WAIT;
    }


    private void shootState() {
        if (rapidFireState.equals(Teleop.SHOOTER_STATE.INACTIVE_STATE))
            rapidFireState = Teleop.SHOOTER_STATE.START_STATE;
        if (previousState.equals(AUTO_STATES.WAIT))
            updateRapidFireStateMachine(3);
        else if (previousState.equals(AUTO_STATES.PATH_TO_SHOOT2))
            updateRapidFireStateMachine(1);

        if (rapidFireState.equals(Teleop.SHOOTER_STATE.END_STATE)) {
            spindex.changeCurrentPositionBy(spindexThirdRevolution/2);
            if (previousState.equals(AUTO_STATES.WAIT)) {
//                currentState = AUTO_STATES.PATH_TO_INTAKE1;
                spindex.resetSpindexToZero();
                currentState = AUTO_STATES.LEAVE;
            } else if (previousState.equals(AUTO_STATES.PATH_TO_SHOOT2))
                currentState = AUTO_STATES.LEAVE;
        }
    }

//    private void shootState() {
//        intake.setPower(.25);
//        shooter.setMotorVelocity(shooterDesiredVelocity);
//        if (shooter.getRightVelocity() > shooterDesiredVelocity * .8) {
//            spindex.changeCurrentPositionBy(spindexThirdRevolution);
//            if (spindex.getColor(spindex.spindexColorBack).equals(GeneralConstants.colorSensorStates.OCCUPIED) && shooter.getRightVelocity() > shooterDesiredVelocity * .95) {
//                currentState = AUTO_STATES.SHOOT_STATE_TWO;
//                shootTimer.reset();
//            }
//        }
//    }
//
//    private void shootStatePart2() {
//        intake.setPower(.25);
//        spindex.stopSpindex();
//        spindex.runTransferWheel();
//        if (rapidFireState.equals(Teleop.SHOOTER_STATE.INACTIVE_STATE) && shootTimer.time() > 3.76) {
//            timesShot += 1;
//            shootTimer.reset();
//            if (timesShot < 3)
//                currentState = AUTO_STATES.SHOOT_STATE;
//            else {
//                timesShot = 0;
//                previousState = AUTO_STATES.SHOOT_STATE_TWO;
//                currentState = AUTO_STATES.PATH_TO_INTAKE1;
//            }
//        }
//    }
    private void pathIntake1() {
        setupPath(intakePath1, intakePose1.getHeading());
        follower.setMaxPower(.5);
        intake.setPower(1);
//        shooter.setMotorVelocity(shooterDesiredVelocity/3);
        currentState = AUTO_STATES.PATH_ACTIVE;
        nextState = AUTO_STATES.PATH_TO_SHOOT2;
    }

    private void pathShoot2() {
        setupPath(shootPath2, shootPose.getHeading());
        spindex.changeCurrentPositionBy(spindexThirdRevolution);
        spindex.changeCurrentPositionBy(spindexThirdRevolution/2);
        spindex.changeCurrentPositionBy(spinAmount);
        rapidFireState = Teleop.SHOOTER_STATE.START_STATE;
        previousState = AUTO_STATES.PATH_TO_SHOOT2;
        //Sort Balls while moving
        currentState = AUTO_STATES.PATH_ACTIVE;
        nextState = AUTO_STATES.SHOOT_STATE;
    }

    private void leave() {
        setupPath(leavePath, leavePose.getHeading());

        currentState = AUTO_STATES.PATH_ACTIVE;
        nextState = AUTO_STATES.END;
    }

    private void pathActiveState() {
        spindex.reverseTransfer();
        shootTimer.reset();
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
            case WAIT: waitState(); break;
            case PATH_ACTIVE: pathActiveState(); break;
            case SHOOT_STATE: shootState(); break;
//            case SHOOT_STATE_TWO: shootStatePart2(); break;
            case PATH_TO_INTAKE1: pathIntake1(); break;
            case PATH_TO_SHOOT2: pathShoot2(); break;
            case LEAVE: leave(); break;
            case INACTIVE: inactiveState(); break;
            case END:
                if (spindex.spindexMotor.getCurrentPosition() < 5)
                    stop();
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        stateMachine();

        telemetryA.addData("rapidfirestate:", rapidFireState);
        telemetryA.addData("current state", currentState);
        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryA.addData("shot timer:", shootTimer.time());
        telemetryA.addData("shooter velocity:", shooter.getRightVelocity());
        telemetryA.addData("spindex position:", spindex.spindexMotor.getCurrentPosition());
        telemetryA.addData("spindex target:", spindex.currentSpindexPosition);
        telemetryA.update();
    }



    private void updateRapidFireStateMachine(int shotCount) {
        switch (rapidFireState) {
            case START_STATE:
                rapidFireState = Teleop.SHOOTER_STATE.RUN_SHOOTER_MOTOR_STATE;
                break;

            case RUN_SHOOTER_MOTOR_STATE:
                shooter.setMotorVelocity(shooterDesiredVelocity);
                spindex.runSpindexToTransferThird();

                //Go to Next State
                rapidFireState = Teleop.SHOOTER_STATE.WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE;

                break;

            case RUN_SPINDEX_STATE:
                spindex.changeCurrentPositionBy(spindexThirdRevolution);
                rapidFireState = Teleop.SHOOTER_STATE.WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE;
                break;

            case WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE:
                spindex.stopTransferWheel();

                //Go to next state when Artifact is in position AND shooter has reached desired velocity
                if (shooter.getRightVelocity() > shooterDesiredVelocity * .95 && Math.abs(spindex.spindexMotor.getCurrentPosition() - Spindex.currentSpindexPosition) < 5) {
//                    if (spindex.getColor(spindex.spindexColorRight).equals(GeneralConstants.colorSensorStates.OCCUPIED))
                        rapidFireState = Teleop.SHOOTER_STATE.RUN_TRANSFER_STATE;
//                    else
//                        rapidFireState = Teleop.SHOOTER_STATE.RUN_SPINDEX_STATE;
                    shootTimer.reset();
                }
                break;

            case RUN_TRANSFER_STATE:
                spindex.runTransferWheel();

                //Repeat RUN_SPINDEX State when timer has reached 3 seconds or when artifact is shot
                if (shootTimer.time(TimeUnit.MILLISECONDS) > 2750 && timesShot < shotCount) {
                    shootTimer.reset();
                    timesShot++;
                    rapidFireState = Teleop.SHOOTER_STATE.RUN_SPINDEX_STATE;
                }

                if (timesShot == shotCount){
                    rapidFireState = Teleop.SHOOTER_STATE.END_STATE;
                }
                break;

            case END_STATE:
                shooter.stop();
                spindex.stopTransferWheel();
                spindex.stopSpindex();
                timesShot = 0;

                rapidFireState = Teleop.SHOOTER_STATE.INACTIVE_STATE;

                break;

            case INACTIVE_STATE:
                break;
        }
    }
}
