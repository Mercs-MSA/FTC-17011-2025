package org.firstinspires.ftc.teamcode.pedroPathingVisualizerTestPrograms;

import static org.firstinspires.ftc.teamcode.Teleop.spindexThirdRevolution;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SoftElectronics;
import org.firstinspires.ftc.teamcode.Teleop;
import org.firstinspires.ftc.teamcode.mechanisms.Drivebase;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Spindex;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "PPV Bezier Red Auto", group = "Autonomous")
@Configurable // Panels
public class PPVBezierRedAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Paths paths; // Paths defined in the Paths class
    private SoftElectronics softElectronics;

    private Spindex spindex;
    private Intake intake;
    private Shooter shooter;
    private ElapsedTime autoTimer;
    private ElapsedTime shooterTimer;
    private int timesShot;
    public static double offsetX = 10;
    public static double offsetY = -7;
    private static Teleop.SHOOTER_STATE rapidFireState = Teleop.SHOOTER_STATE.INACTIVE_STATE;
    private Drivebase drivebase;
    private static double lastShooterVelocity;
    public static double shooterVelocityDropThreshold = 80.6741;



    private enum AUTO_STATE {
        INIT,
        PATH_ACTIVE_WAIT,
        startToShootFar,
        intakeLevel1Ball1,
        intakeLevel1Ball2,
        intakeLevel1Ball3,
        intakeLevel1ToShootFar,
        shootFarToLeave,
        intakeLevel2Ball1,
        intakeLevel2Ball2,
        intakeLevel2Ball3,
        intakeLevel2ToShootClose,
        intakeLevel3Ball1,
        intakeLevel3Ball2,
        intakeLevel3Ball3,
        intakeLevel3ToShootClose,
        parkByGate,
        INACTIVE,
        FULL_ROTATE_SPINDEX,
        HALF_ROTATE_SPINDEX,
        SHOOT,
        WAIT_UNTIL_SHOOT_DONE,
        WAIT_UNTIL_WAIT_DONE,
        WAIT_STATE,
        LEAVE,
        END
    }

    private AUTO_STATE PAW_NextState; //Path Active Wait Next State
    private AUTO_STATE FR_NextState; //Full Rotate Spindex Next State
    private AUTO_STATE HR_NextState; //Half Rotate Spindex Next State
    private AUTO_STATE S_NextState; //Full Rotate Spindex Next State
    private AUTO_STATE W_NextState; //Full Rotate Spindex Next State
    private double W_StateMilliseconds = 0.67;



    private AUTO_STATE autoState;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        softElectronics = new SoftElectronics(hardwareMap, this.telemetry);

        drivebase = new Drivebase(hardwareMap);
        spindex = new Spindex(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        autoTimer = new ElapsedTime();
        timesShot = 0;

        shooterTimer = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        lastShooterVelocity = shooter.getRightVelocity();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        telemetry.addData("Current Robot Position:", "67");

        autoState = AUTO_STATE.INIT;
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autoState = autonomousPathUpdate(); // Update autonomous state machine
        updateRapidFireStateMachine();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Auto State", autoState);
        panelsTelemetry.debug("Shooter State:", rapidFireState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain startToShootFar;
        public PathChain intakeLevel1Ball1;
        public PathChain intakeLevel1Ball2;
        public PathChain intakeLevel1Ball3;
        public PathChain intakeLevel1ToShootFar;
        public PathChain shootFarToLeave;
        public PathChain intakeLevel2Ball1;
        public PathChain intakeLevel2Ball2;
        public PathChain intakeLevel2Ball3;
        public PathChain intakeLevel2ToShootClose;
        public PathChain intakeLevel3Ball1;
        public PathChain intakeLevel3Ball2;
        public PathChain intakeLevel3Ball3;
        public PathChain intakeLevel3ToShootClose;
        public PathChain parkByGate;
        public PathChain LEAVE;

        public Paths(Follower follower) {
            startToShootFar = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.000, 8.000),
                                    new Pose(96.210, 17.938 + offsetY),
                                    new Pose(86.261, 20.723 + offsetY)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))
                    .build();

            LEAVE = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(86.261, 20.723 + offsetY),
                                    new Pose(86.261, 35.723 + offsetY)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(90))
                    .build();

            intakeLevel1Ball1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(84.261 - 15, 21.723 + offsetY - 15),
                                    new Pose(78.007, 38.016),
                                    new Pose(108.946 - 15, 35.383 - 15)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))
                    .build();

            intakeLevel1Ball2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(108.946 - 15, 35.383 - 15),
                                    new Pose(114.048 - 15, 35.383 - 15))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intakeLevel1Ball3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(114.048 - 15, 35.383 - 15),
                                    new Pose(119.500 - 15, 35.383 - 15))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intakeLevel1ToShootFar = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(119.500 - 15, 35.383 - 15),
                                    new Pose(103.845, 23.698),
                                    new Pose(86.261, 20.723 + offsetY)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))
                    .build();

            shootFarToLeave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(86.261, 20.723 + offsetY),
                                    new Pose(84.261, 35.383)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(90))
                    .build();

            }
        }

/*
        public Paths(Follower follower) {
            startToShootFar = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144 - (56.000), 8.000), // 88.0, 8.0
                                    new Pose(144 - (55.790), 17.938), // 88.21, 17.938
                                    new Pose(144 - (59.739 + offsetX), 21.723) // 144 - (69.739) = 74.261
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70)) // 90 → 180-110=70
                    .build();

            intakeLevel1Ball1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144 - (59.739 + offsetX), 21.723), // 74.261,21.723
                                    new Pose(144 - (65.993 + offsetX), 38.016 + offsetY), // 144-(75.993)=68.007
                                    new Pose(144 - (35.054 + offsetX - 6), 35.383 + offsetY) // 144-(44.054)=99.946
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0)) // 115→180-115=65, 180→0
                    .build();

            intakeLevel1Ball2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(144 - (35.054 + offsetX - 6), 35.383 + offsetY), // 99.946
                                    new Pose(144 - (29.952 + offsetX - 6), 35.383 + offsetY)  // 144-(29.952+4)=110.048
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intakeLevel1Ball3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(144 - (29.952 + offsetX - 6), 35.383 + offsetY), // 110.048
                                    new Pose(144 - (24.500 + offsetX - 12), 35.383 + offsetY) // 144-(22.5)=121.5
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intakeLevel1ToShootFar = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144 - (24.500 + offsetX - 12), 35.383 + offsetY), // 121.5
                                    new Pose(144 - (40.155 + offsetX), 23.698 + offsetY), // 144-(50.155)=93.845
                                    new Pose(144 - (59.739 + offsetX), 21.723) // 74.261
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(115)) // 180→0, 115→65
                    .build();

            shootFarToLeave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144 - (59.739 + offsetX), 21.723), // 74.261
                                    new Pose(144 - (59.739 + offsetX), 35.383 + offsetY) // 74.261
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(90))
                    .build();
        }

 */


    private void updateRapidFireStateMachine() {

        switch (rapidFireState) {
            case START_STATE:
                spindex.changeCurrentPositionBy(spindexThirdRevolution/2.0);
                timesShot = 0;
                rapidFireState = Teleop.SHOOTER_STATE.RUN_SHOOTER_MOTOR_STATE;
                break;

            case RUN_SHOOTER_MOTOR_STATE:
                shooter.setMotorVelocity(1550);

                //Go to Next State
                rapidFireState = Teleop.SHOOTER_STATE.RUN_SPINDEX_STATE;
                break;

            case POINT_AT_GOAL_STATE:

//                double currentHeading = Math.toDegrees(drivebase.otos.getPosition().h + drivebase.getOffset());
//
//                // smallest rotation
//                double error = ((shooterDesiredVelocity == closeZoneVelocity ? closeZoneHeading : farZoneHeading) - currentHeading) * -1;
//
//                double turnPower = error * drivebase.kP;
//
//                if (turnPower > 0) {
//                    turnPower = Math.min(turnPower, 0.267);
//                } else if (turnPower < 0) {
//                    turnPower = Math.max(turnPower, -0.267);
//                }
//                double headingError = currentHeading - (shooterDesiredVelocity == farZoneHeading ? farZoneHeading : closeZoneHeading) ;
//
//                if (Math.abs(headingError) < 2.0) {   // robot is basically facing the target
//                    drivebase.stop();  // stop motors
//                    rapidFireState = Teleop.SHOOTER_STATE.RUN_SPINDEX_STATE;
//                }
//
//                drivebase.setDrivePower(turnPower, -turnPower, turnPower, -turnPower);


                break;

            case RUN_SPINDEX_STATE:
                spindex.changeCurrentPositionBy(spindexThirdRevolution);
                rapidFireState = Teleop.SHOOTER_STATE.WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE;
                break;

            case WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE:

                if (shooter.getRightVelocity() > 1550 * .97 && shooter.getLeftVelocity() > 1550 * .97 && !spindex.isSpindexMoving()) {
                    rapidFireState = Teleop.SHOOTER_STATE.RUN_TRANSFER_STATE;
                    shooterTimer.reset();
                }

                break;

            case RUN_TRANSFER_STATE:
                spindex.runTransferWheel();

                //Repeat RUN_SPINDEX State when timer has reached 3 seconds or when artifact is shot
                if (lastShooterVelocity - shooter.getRightVelocity() > shooterVelocityDropThreshold || shooterTimer.time(TimeUnit.MILLISECONDS) > 1200) {
//                    rapidFireTimer.reset();
//                    numOfBallsInRobot--;
//                    if (numOfBallsInRobot < 0)
//                        numOfBallsInRobot = 0;
                    timesShot++;
                    if (timesShot >= 3) {
                        rapidFireState = Teleop.SHOOTER_STATE.END_STATE;
                    } else {
                        rapidFireState = Teleop.SHOOTER_STATE.RUN_SPINDEX_STATE;
                    }
                }

                lastShooterVelocity = shooter.getRightVelocity();

                break;

            case END_STATE:
                shooter.stop();
                spindex.stopTransferWheel();
                spindex.changeCurrentPositionBy(spindexThirdRevolution/2.0);

                rapidFireState = Teleop.SHOOTER_STATE.INACTIVE_STATE;

                break;

            case INACTIVE_STATE:
                break;
        }
    }

    public AUTO_STATE autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine

        switch (autoState) {
            /// General Cases
            case PATH_ACTIVE_WAIT:
                if (!follower.isBusy()) {
                    autoState = PAW_NextState;
                    autoTimer.reset();
                }

                break;

            case FULL_ROTATE_SPINDEX:
                spindex.changeCurrentPositionBy(spindexThirdRevolution);

                autoState = FR_NextState;
                autoTimer.reset();
                break;

            case HALF_ROTATE_SPINDEX:
                spindex.changeCurrentPositionBy(spindexThirdRevolution/2.0);

                autoState = HR_NextState;
                autoTimer.reset();
                break;

            case SHOOT:
                rapidFireState = Teleop.SHOOTER_STATE.START_STATE;

                autoState = AUTO_STATE.WAIT_UNTIL_SHOOT_DONE;
                autoTimer.reset();
                break;

            case WAIT_UNTIL_SHOOT_DONE:
                if (rapidFireState.equals(Teleop.SHOOTER_STATE.INACTIVE_STATE))
                    autoState = S_NextState;
                autoTimer.reset();

                break;

            case WAIT_STATE:
                autoTimer.reset();
                autoState = AUTO_STATE.WAIT_UNTIL_WAIT_DONE;
                break;

            case WAIT_UNTIL_WAIT_DONE:
                if (autoTimer.time(TimeUnit.MILLISECONDS) > W_StateMilliseconds)
                    autoState = W_NextState;

                break;


            /// Ordered Cases
            case startToShootFar:
                follower.setMaxPower(0.75);
                follower.followPath(paths.startToShootFar);

                PAW_NextState = AUTO_STATE.SHOOT;
                S_NextState = AUTO_STATE.intakeLevel1Ball1;

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;

            case LEAVE:
                follower.followPath(paths.LEAVE);

                PAW_NextState = AUTO_STATE.END;

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;


            case intakeLevel1Ball1:
                intake.setPower(1);
                follower.setMaxPower(.25);
                follower.followPath(paths.intakeLevel1Ball1);

                PAW_NextState = AUTO_STATE.WAIT_STATE;
                W_StateMilliseconds = 1000.67;

                W_NextState = AUTO_STATE.FULL_ROTATE_SPINDEX;

                FR_NextState = AUTO_STATE.intakeLevel1Ball2;

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;

            case intakeLevel1Ball2:
                if (autoTimer.time(TimeUnit.MILLISECONDS) > 1000) {
                    follower.followPath(paths.intakeLevel1Ball2);

                    PAW_NextState = AUTO_STATE.WAIT_STATE;
                    W_StateMilliseconds = 1000.67;

                    W_NextState = AUTO_STATE.FULL_ROTATE_SPINDEX;

                    FR_NextState = AUTO_STATE.intakeLevel1Ball3;

                    autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                }
                break;

            case intakeLevel1Ball3:
                if (autoTimer.time(TimeUnit.MILLISECONDS) > 1000) {
                    follower.followPath(paths.intakeLevel1Ball3);

                    PAW_NextState = AUTO_STATE.intakeLevel1ToShootFar;

                    autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                }
                break;

            case intakeLevel1ToShootFar:
                if (autoTimer.time(TimeUnit.MILLISECONDS) > 1000) {
                    intake.setPower(0);
                    follower.setMaxPower(.6);
                    follower.followPath(paths.intakeLevel1ToShootFar);

                    PAW_NextState = AUTO_STATE.SHOOT;
                    S_NextState = AUTO_STATE.shootFarToLeave;

                    autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                }
                break;

            case shootFarToLeave:
                follower.setMaxPower(1);
                follower.followPath(paths.shootFarToLeave);

                PAW_NextState = AUTO_STATE.END;

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;

                break;

            /*
            case intakeLevel2Ball1:
                intake.setPower(1);
                follower.setMaxPower(.5);
                follower.followPath(paths.intakeLevel2Ball1);

                PAW_NextState = AUTO_STATE.FULL_ROTATE_SPINDEX;
                FR_NextState = AUTO_STATE.intakeLevel2Ball2;

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;

            case intakeLevel2Ball2:
                follower.followPath(paths.intakeLevel2Ball2);

                PAW_NextState = AUTO_STATE.FULL_ROTATE_SPINDEX;
                FR_NextState = AUTO_STATE.intakeLevel2Ball3;

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;

            case intakeLevel2Ball3:
                follower.followPath(paths.intakeLevel2Ball3);

                PAW_NextState = AUTO_STATE.FULL_ROTATE_SPINDEX;
                FR_NextState = AUTO_STATE.intakeLevel2ToShootClose;

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;

            case intakeLevel2ToShootClose:
                intake.setPower(0);
                shooterVelocity = Teleop.closeZoneVelocity;
                follower.followPath(paths.intakeLevel2ToShootClose);

                spindex.changeCurrentPositionBy(spindexThirdRevolution);
                PAW_NextState = AUTO_STATE.SHOOT;
                S_NextState = AUTO_STATE.HALF_ROTATE_SPINDEX;
                HR_NextState = AUTO_STATE.intakeLevel3Ball1;

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;

            case intakeLevel3Ball1:
                intake.setPower(1);
                follower.followPath(paths.intakeLevel3Ball1);

                PAW_NextState = AUTO_STATE.FULL_ROTATE_SPINDEX;
                FR_NextState = AUTO_STATE.intakeLevel3Ball2;

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;

            case intakeLevel3Ball2:
                follower.followPath(paths.intakeLevel3Ball2);

                PAW_NextState = AUTO_STATE.FULL_ROTATE_SPINDEX;
                FR_NextState = AUTO_STATE.intakeLevel3Ball3;

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;

            case intakeLevel3Ball3:
                follower.followPath(paths.intakeLevel3Ball3);

                PAW_NextState = AUTO_STATE.FULL_ROTATE_SPINDEX;
                FR_NextState = AUTO_STATE.intakeLevel3ToShootClose;

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;
            case intakeLevel3ToShootClose:
                intake.setPower(0);
                follower.followPath(paths.intakeLevel3ToShootClose);

                spindex.changeCurrentPositionBy(spindexThirdRevolution);
                PAW_NextState = AUTO_STATE.SHOOT;
                S_NextState = AUTO_STATE.END;

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;
             */

            case INIT:
                autoState = AUTO_STATE.startToShootFar;
                break;

            case END:
                break;

        }

        return autoState;
    }
}
