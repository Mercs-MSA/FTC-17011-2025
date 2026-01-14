package org.firstinspires.ftc.teamcode.Autos;

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

import org.firstinspires.ftc.teamcode.mechanisms.Drivebase;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "PPV Red Far Auto", group = "Autonomous")
@Configurable // Panels
public class PPVRedFarAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Drivebase drivebase;
    private Transfer transfer;
    private Intake intake;
    private Shooter shooter;
    private ElapsedTime autoTimer;
    private double lastVelocity;
    private int timesShot = 0;
    private double shooterVelocityDropThreshold = 80.6741;

    private enum AUTO_STATE {
        startToShootFar,
        shootToIntakeLevel1,
        intakeLevel1ToShoot,
        shootToIntakeHuman1,
        intakeHuman1to2,
        intakeHumanToShoot,
        INIT,
        PATH_ACTIVE_WAIT,
        SHOOT,
        WAIT_UNTIL_WAIT_DONE,
        WAIT,
        LEAVE,
        END,
        NOTHING,
    }

    private enum SHOOTING_STATE {
        INACTIVE,
        START,
        START_SHOOTER,
        IS_SHOOTER_READY,
        SHOOT_BALL,
        END
    }
    public static SHOOTING_STATE shootingState = SHOOTING_STATE.INACTIVE;

    private AUTO_STATE P_NextState; //Path Next State
    private AUTO_STATE S_NextState; //Shooter Next State
    private AUTO_STATE W_NextState; //Wait Next State
    private double W_NextStateLengthMS = 670.0;
    private double time = 0;
    private AUTO_STATE autoState = AUTO_STATE.INIT;
    private Paths paths; // Paths defined in the Paths class

    private double shooterDesiredVelocity = org.firstinspires.ftc.teamcode.Constants.Constants.FAR_SHOT_VELOCITY;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        autoState = AUTO_STATE.INIT;
        P_NextState = AUTO_STATE.NOTHING;
        W_NextState = AUTO_STATE.NOTHING;
        S_NextState = AUTO_STATE.NOTHING;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        drivebase = new Drivebase(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        autoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        lastVelocity = shooterDesiredVelocity;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        time = autoTimer.time();
        follower.update(); // Update Pedro Pathing
        updateAutoStateMachine(); // Update autonomous state machine
        updateShooterStateMachine();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Auto State", autoState);
        panelsTelemetry.debug("P_NextState", P_NextState);
        panelsTelemetry.debug("W_NextState", W_NextState);
        panelsTelemetry.debug("S_NextState", S_NextState);
        panelsTelemetry.addLine("\n");
        panelsTelemetry.debug("Shooter State", shootingState);
        panelsTelemetry.debug("Shooter Velocity: ", shooter.getVelocity());
        panelsTelemetry.debug("Shooter Desired Velocity: ", shooterDesiredVelocity);
        panelsTelemetry.debug("Auto Timer: ", time);


        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain startToShootFar;
        public PathChain shootToIntakeLevel1;
        public PathChain intakeLevel1ToShoot;
        public PathChain shootToIntakeHuman1;
        public PathChain intakeHuman1to2;
        public PathChain intakeHumanToShoot;
        public PathChain shootToGate1;
        public PathChain gateToShoot1;
        public PathChain shootToGate2;
        public PathChain gateToShoot2;
        public PathChain leave;

        public Paths(Follower follower) {
            startToShootFar = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 8.000),
                                    new Pose(88.210, 17.938),
                                    new Pose(84.261, 21.723)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))

                    .build();

            shootToIntakeLevel1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.261, 21.723),
                                    new Pose(89.477, 38.680),
                                    new Pose(101.474, 28.000),
                                    new Pose(125.455, 35.302)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))

                    .build();

            intakeLevel1ToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.455, 35.302),

                                    new Pose(84.261, 21.723)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))

                    .build();

            shootToIntakeHuman1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.261, 21.723),
                                    new Pose(132.581, 29.561),
                                    new Pose(132.746, 16.446)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(-30))

                    .build();

            intakeHuman1to2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.746, 16.446),

                                    new Pose(133.135, 10.769)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-30))

                    .build();

            intakeHumanToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(133.135, 10.769),
                                    new Pose(116.872, 22.769),
                                    new Pose(84.261, 21.723)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(65))

                    .build();

            shootToGate1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.261, 21.723),

                                    new Pose(132.684, 32.526)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(90))

                    .build();

            gateToShoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.684, 32.526),

                                    new Pose(84.261, 21.723)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))

                    .build();

            shootToGate2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.261, 21.723),

                                    new Pose(132.579, 10.842)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(-30))

                    .build();

            gateToShoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.579, 10.842),

                                    new Pose(84.211, 21.895)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(65))

                    .build();

            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.211, 21.895),

                                    new Pose(90.053, 26.737)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(90))

                    .build();
        }
    }

    private boolean desiredVelocityReached() {
        return (shooter.getVelocity() > shooterDesiredVelocity * .97);
    }

    public void updateAutoStateMachine() {
        switch (autoState) {
            case PATH_ACTIVE_WAIT:
                if (!follower.isBusy()) {
                    autoState = P_NextState;
                }
                break;

            case SHOOT:
                if (shootingState.equals(SHOOTING_STATE.INACTIVE) && follower.atPose(new Pose(84.26057142857142, 21.723), 2.25, 2.25) && (follower.getHeading() < Math.toRadians(69)) && (follower.getHeading() > Math.toRadians(61))) {
                    autoTimer.reset();
                    shootingState = SHOOTING_STATE.START;
                } else if (shootingState.equals(SHOOTING_STATE.END)) {
                    shootingState = SHOOTING_STATE.INACTIVE;
                    autoState = S_NextState;
                }

            case WAIT:
//                autoTimer.reset();
                //autoState = AUTO_STATE.WAIT_UNTIL_WAIT_DONE;
                break;

            case WAIT_UNTIL_WAIT_DONE:
                if (time > W_NextStateLengthMS)
                    autoState = W_NextState;

                break;

            case INIT:
                autoState = AUTO_STATE.startToShootFar;
                break;

            case startToShootFar:
                shooter.setMotorVelocity(1800);
                shooter.setTurretTarget(23.8);
                follower.followPath(paths.startToShootFar);
                P_NextState = AUTO_STATE.SHOOT;
                S_NextState = AUTO_STATE.shootToIntakeLevel1;


                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;

            case shootToIntakeLevel1:
                intake.setPower(1);
                follower.followPath(paths.shootToIntakeLevel1);
                P_NextState = AUTO_STATE.intakeLevel1ToShoot;

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;

            case intakeLevel1ToShoot:
                follower.followPath(paths.intakeLevel1ToShoot);
                P_NextState = AUTO_STATE.SHOOT;
                S_NextState = AUTO_STATE.shootToIntakeHuman1;

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;

            case shootToIntakeHuman1:
                follower.followPath(paths.intakeLevel1ToShoot);
                P_NextState = AUTO_STATE.intakeHuman1to2;

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;

            case intakeHuman1to2:
                follower.followPath(paths.shootToIntakeHuman1);
                P_NextState = AUTO_STATE.intakeHumanToShoot;

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;

            case intakeHumanToShoot:
                follower.followPath(paths.intakeHumanToShoot);
                P_NextState = AUTO_STATE.SHOOT;
                S_NextState = AUTO_STATE.END;

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;

            case END:
                shooter.setMotorVelocity(0);
                intake.setPower(0);
                break;

        }
    }
    private void updateShooterStateMachine() {
        switch (shootingState) {
            case START:
                shootingState = SHOOTING_STATE.START_SHOOTER;
                break;

            case START_SHOOTER:
//                timesShot = 0;
                shooter.setMotorVelocity(shooterDesiredVelocity);
                transfer.closeTransferGate();

                shootingState = SHOOTING_STATE.IS_SHOOTER_READY;
                break;

            case IS_SHOOTER_READY:
                if (time > 2800.0) {
                    shootingState = SHOOTING_STATE.END;
                }
                if (desiredVelocityReached()) {
                    shootingState = SHOOTING_STATE.SHOOT_BALL;
                }
                break;

            case SHOOT_BALL:
                transfer.openTransferGate();
                transfer.setPower(.87);
                if (!desiredVelocityReached()) {
                    if (time > 2800.0) {
                        shootingState = SHOOTING_STATE.END;
                    } else {
                        shootingState = SHOOTING_STATE.IS_SHOOTER_READY;
                    }
                }
                if (time > 2800.0)
                    shootingState = SHOOTING_STATE.END;
                break;

            case END:
                shooter.setMotorVelocity(800);
                transfer.closeTransferGate();
                transfer.setPower(0);
//                shootingState = SHOOTING_STATE.INACTIVE;
                break;

            case INACTIVE:
                transfer.closeTransferGate();
                break;
        }
    }
}
