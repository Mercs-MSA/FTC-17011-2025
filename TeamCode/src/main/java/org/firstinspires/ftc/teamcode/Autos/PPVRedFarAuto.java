package org.firstinspires.ftc.teamcode.Autos;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Teleop;
import org.firstinspires.ftc.teamcode.mechanisms.Drivebase;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import static org.firstinspires.ftc.teamcode.Constants.Constants.blueGoal;
import static org.firstinspires.ftc.teamcode.Constants.Constants.currentTheta;
import static org.firstinspires.ftc.teamcode.Constants.Constants.currentX;
import static org.firstinspires.ftc.teamcode.Constants.Constants.currentY;
import static org.firstinspires.ftc.teamcode.Constants.Constants.ranAuto;
import static org.firstinspires.ftc.teamcode.Constants.Constants.redGoal;
import static org.firstinspires.ftc.teamcode.Constants.Constants.onBlueAlliance;


@Autonomous(name = "Red Far 9", group = "Red")
@Configurable
public class PPVRedFarAuto extends OpMode {


    private TelemetryManager panelsTelemetry;
    private FtcDashboard dash;


    private MultipleTelemetry myTelem;


    public Follower follower;


    private ColorRangeSensor innerColor;


    private Drivebase drivebase;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;


    private ElapsedTime autoTimer;
    private ElapsedTime shooterTimer;


    private double shooterDesiredVelocity = 0;


    private enum AUTO_STATE {
        startToShootFar,
        shootToIntakeLevel1,
        intakeLevel1ToShoot,
        shootToIntakeHuman1,
        intakeHuman1to2,
        intakeHumanToShoot,
        leave,
        INIT,
        PATH_ACTIVE_WAIT,
        SHOOT,
        END
    }


    private enum SHOOTING_STATE {
        INACTIVE,
        START,
        START_SHOOTER,
        IS_SHOOTER_READY,
        SHOOT_BALL,
        END
    }


    private enum TURRET_STATE {
        GO_TO_ZERO,
        ZEROED,
        AIMED,
        AIMING_NO_TAG,
        // AIMING_TO_TAG
    }


    private AUTO_STATE autoState = AUTO_STATE.INIT;
    private AUTO_STATE P_NextState;
    private AUTO_STATE S_NextState;


    public static SHOOTING_STATE shootingState = SHOOTING_STATE.INACTIVE;


    private TURRET_STATE turretState = TURRET_STATE.AIMING_NO_TAG;
    // private TURRET_STATE turretState = TURRET_STATE.AIMING_TO_TAG;


    private Paths paths;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));


        paths = new Paths(follower);


        innerColor = hardwareMap.get(ColorRangeSensor.class, "innerColor");


        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap, 0);


        autoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        shooterTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


        dash = FtcDashboard.getInstance();
        myTelem = new MultipleTelemetry(dash.getTelemetry(), telemetry);

        ranAuto = true;
        onBlueAlliance = false;

        follower.setMaxPower(.5);
        autoState = AUTO_STATE.INIT;
        shootingState = SHOOTING_STATE.INACTIVE;
//        panelsTelemetry.debug("Status", "Initialized");
//        panelsTelemetry.update(telemetry);
        myTelem.addLine("Status Initialized");
        myTelem.update();
    }


    @Override
    public void loop() {
        follower.update();
        currentX = follower.getPose().getX();
        currentY = follower.getPose().getY();
        currentTheta = follower.getPose().getHeading();
        setShooterDesiredVelocity();
        updateAutoStateMachine();
        updateShooterStateMachine();
        if (!autoState.equals(AUTO_STATE.leave) ^ autoState.equals(AUTO_STATE.END))
            updateTurretState();


//        panelsTelemetry.update(telemetry);
        myTelem.addData("Shooter Velocity: ", shooter.getVelocity());
        myTelem.addData("Shooter Target Velocity: ", shooterDesiredVelocity);
        myTelem.addData("Auto State: ", autoState);
        myTelem.addData("Shooter State: ", shootingState);
        myTelem.update();
    }


    private void updateAutoStateMachine() {
        switch (autoState) {


            case PATH_ACTIVE_WAIT:
                if (!follower.isBusy())
                    autoState = P_NextState;
                break;


            case SHOOT:
                if (shootingState.equals(SHOOTING_STATE.INACTIVE)) {
                    autoTimer.reset();
                    shootingState = SHOOTING_STATE.START;
                } else if (shootingState.equals(SHOOTING_STATE.END)) {
                    shootingState = SHOOTING_STATE.INACTIVE;
                    autoState = S_NextState;
                }
                break;


            case INIT:
                autoState = AUTO_STATE.startToShootFar;
                turretState = TURRET_STATE.AIMING_NO_TAG;
                break;


            case startToShootFar:
                shooter.setMotorVelocity(shooterDesiredVelocity);


                follower.followPath(paths.startToShootFar);
                P_NextState = AUTO_STATE.SHOOT;
                S_NextState = AUTO_STATE.shootToIntakeLevel1;
                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;


            case shootToIntakeLevel1:
                follower.setMaxPower(.8);
                intake.setPower(1);
                follower.followPath(paths.shootToIntakeLevel1);
                P_NextState = AUTO_STATE.intakeLevel1ToShoot;
                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;


            case intakeLevel1ToShoot:
                follower.setMaxPower(.6);
                follower.followPath(paths.intakeLevel1ToShoot);
                P_NextState = AUTO_STATE.SHOOT;
                S_NextState = AUTO_STATE.shootToIntakeHuman1;
                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;


            case shootToIntakeHuman1:
                follower.setMaxPower(.8);
                follower.followPath(paths.shootToIntakeHuman1);
                P_NextState = AUTO_STATE.intakeHuman1to2;
                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;


            case intakeHuman1to2:
                follower.setMaxPower(.6);
                follower.followPath(paths.intakeHuman1to2);
                P_NextState = AUTO_STATE.intakeHumanToShoot;
                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;


            case intakeHumanToShoot:
                follower.setMaxPower(.5);
                follower.followPath(paths.intakeHumanToShoot);
                P_NextState = AUTO_STATE.SHOOT;
//                S_NextState = AUTO_STATE.END;
                S_NextState = AUTO_STATE.leave;
                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;


            case leave:
                follower.followPath(paths.leave);
                P_NextState = AUTO_STATE.END;
                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
            case END:
                shooter.setMotorVelocity(0);
                intake.setPower(0);
                break;
        }
    }


    public double distanceToTarget() {
        double range = 0;
        if (onBlueAlliance) {
            range = Math.hypot(follower.getPose().getX() - blueGoal.x, follower.getPose().getY() - blueGoal.y);
        } else {
            range = Math.hypot(follower.getPose().getX() - redGoal.x, follower.getPose().getY() - redGoal.y);
        }
        return range;
    }


    private void setShooterDesiredVelocity() {
        double range = distanceToTarget();
        int velocity = 0;
        if (range < 80)
            velocity = (int) ((0.0586009 * Math.pow(range, 2)) + (-4.2766 * range) + 1498.02814);
        else
            velocity = (int) ((0.0227675 * Math.pow(range, 2)) + (1.62765 * range) + 1344.59538);


        shooterDesiredVelocity = Math.min(velocity, 1930);
    }


    private boolean desiredVelocityReached() {
        return shooter.getVelocity() > shooterDesiredVelocity * .98;
    }


    private void updateShooterStateMachine() {
        switch (shootingState) {


            case START:
                shootingState = SHOOTING_STATE.START_SHOOTER;
                break;


            case START_SHOOTER:
                shooter.setMotorVelocity(shooterDesiredVelocity);
                transfer.closeTransferGate();
                shooterTimer.reset();
                shootingState = SHOOTING_STATE.IS_SHOOTER_READY;
                break;


            case IS_SHOOTER_READY:
                transfer.closeTransferGate();
                if (shooterTimer.time() > 2800)
                    shootingState = SHOOTING_STATE.END;
                if (desiredVelocityReached()) {
//                    shooterTimer.reset();
                    shootingState = SHOOTING_STATE.SHOOT_BALL;
                }
                break;


            case SHOOT_BALL:
                transfer.openTransferGate();
                transfer.setPower(0.87);
                if (shooterTimer.time() > 2800)
                    shootingState = SHOOTING_STATE.END;
//                if (innerColor.getDistance(DistanceUnit.INCH) > 3 && shooterTimer.time() > 500)
//                    shootingState = SHOOTING_STATE.END;
                if (!desiredVelocityReached())
                    shootingState = SHOOTING_STATE.IS_SHOOTER_READY;
                break;


            case END:
                transfer.closeTransferGate();
                transfer.setPower(0);
                break;


            case INACTIVE:
                transfer.closeTransferGate();
                break;
        }
    }


    private void updateTurretState() {
        Pose pose = follower.getPose();


        double targetFieldAngle = onBlueAlliance
                ? Math.toDegrees(Math.atan2(blueGoal.y - pose.getY(), blueGoal.x - pose.getX()))
                : Math.toDegrees(Math.atan2(redGoal.y - pose.getY(), redGoal.x - pose.getX()));


        double robotHeadingDeg = Math.toDegrees(pose.getHeading());
        double turretSetpointDeg = targetFieldAngle - robotHeadingDeg;
        double turretAngleDeg = shooter.getTurretPos() / 8.13333333333;


        myTelem.addData("Turret Angle: ", turretAngleDeg);
        myTelem.addData("Target Angle: ", turretSetpointDeg);


        double error = AngleUnit.normalizeDegrees(turretSetpointDeg - turretAngleDeg);


//        boolean seesTarget = drivebase.getTargetSeen();
        boolean withinAngleLimit =
                Math.abs(turretAngleDeg) < 112 &&
                        Math.abs(targetFieldAngle) < 112;


        switch (turretState) {


            case GO_TO_ZERO:
                shooter.setTurretVelocity(200, 1);
                shooter.setTurretTarget(0);
                if (Math.abs(shooter.getTurretPos()) < 10)
                    turretState = TURRET_STATE.ZEROED;
                break;


            case ZEROED:
                shooter.setTurretVelocity(0, 0);
                // if (seesTarget)
                //     turretState = TURRET_STATE.AIMING_TO_TAG;
                turretState = TURRET_STATE.AIMING_NO_TAG;
                break;


            case AIMED:
                shooter.setTurretVelocity(0, 0);


                if (gamepad1.left_trigger > .3) {
                    turretState = TURRET_STATE.GO_TO_ZERO;
//                } else if (seesTarget && Math.abs(drivebase.getLLResult().getTx()) > 1) {
//                    turretState = TURRET_STATE.AIMING_TO_TAG;
                } else if (withinAngleLimit && Math.abs(error) > .5) {
                    turretState = TURRET_STATE.AIMING_NO_TAG;
                }
                break;


            case AIMING_NO_TAG:
                if (Math.abs(error) > .5)
                    shooter.setTurretTargetShortestPath(turretSetpointDeg);
                else
                    turretState = TURRET_STATE.AIMED;
                break;


           /*
           case AIMING_TO_TAG:
               shooter.setTurretMode(DcMotor.RunMode.RUN_USING_ENCODER);


               if (!seesTarget) {
                   turretState = TURRET_STATE.AIMING_NO_TAG;
                   return;
               }


               double limeError =
                       AngleUnit.normalizeDegrees(drivebase.getLLResult().getTx());


               if (Math.abs(limeError) > 1.75) {
                   if (!withinAngleLimit)
                       shooter.setTurretVelocity(0, 0);
                   else
                       shooter.setTurretVelocity((int) (-limeError * 100 + 25), 1);
               } else {
                   turretState = TURRET_STATE.AIMED;
               }
               break;
           */
        }
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
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                    .build();

            shootToIntakeLevel1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.261, 21.723),
                                    new Pose(89.477, 38.680),
                                    new Pose(101.474, 28.000),
                                    new Pose(125.455, 35.302)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            intakeLevel1ToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.455, 35.302),

                                    new Pose(84.261, 21.723)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            shootToIntakeHuman1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.261, 21.723),
                                    new Pose(132.581, 29.561),
                                    new Pose(136.000, 17.384)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))

                    .build();

            intakeHuman1to2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(136.000, 17.384),
                                    new Pose(124.506, 16.236),
                                    new Pose(136.000, 8.878)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))

                    .build();

            intakeHumanToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(136.000, 8.878),
                                    new Pose(116.872, 22.769),
                                    new Pose(84.261, 21.723)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            shootToGate1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.261, 21.723),

                                    new Pose(132.684, 32.526)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                    .build();

            gateToShoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.684, 32.526),

                                    new Pose(84.261, 21.723)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                    .build();

            shootToGate2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.261, 21.723),

                                    new Pose(132.700, 9.272)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))

                    .build();

            gateToShoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.700, 9.272),

                                    new Pose(84.211, 21.895)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))

                    .build();

            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.261, 21.723),

                                    new Pose(90.053, 26.737)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                    .build();
        }
    }
}

