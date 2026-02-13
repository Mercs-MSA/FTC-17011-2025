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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Drivebase;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.Constants.Constants.blueGoal;
import static org.firstinspires.ftc.teamcode.Constants.Constants.currentTheta;
import static org.firstinspires.ftc.teamcode.Constants.Constants.currentX;
import static org.firstinspires.ftc.teamcode.Constants.Constants.currentY;
import static org.firstinspires.ftc.teamcode.Constants.Constants.ranAuto;
import static org.firstinspires.ftc.teamcode.Constants.Constants.redGoal;
import static org.firstinspires.ftc.teamcode.Constants.Constants.onBlueAlliance;
import static org.firstinspires.ftc.teamcode.Constants.Constants.turretLastAutoPos;


@Autonomous(name = "Red Close 9 Gate", group = "Red")
@Configurable
public class PPVRedGoalSideAuto extends OpMode{
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private FtcDashboard dash;


    private MultipleTelemetry myTelem;


    private ColorRangeSensor innerColor;


    private Drivebase drivebase;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;


    private ElapsedTime autoTimer;
    private ElapsedTime shooterTimer;


    private double shooterDesiredVelocity = 0;


    private enum AUTO_STATE {
        startToShoot,
        shootToIntake,
        intake1ToGate,
        gateToShoot,
        shootToIntake2,
        intake2Finish,
        intake2ToShoot,
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
        follower.setStartingPose(new Pose(126.177, 119.039, Math.toRadians(36)));

        paths = new Paths(follower);


        innerColor = hardwareMap.get(ColorRangeSensor.class, "innerColor");
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap, 0);


        dash = FtcDashboard.getInstance();
        myTelem = new MultipleTelemetry(dash.getTelemetry(), telemetry);


        autoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        shooterTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        ranAuto = true;
        onBlueAlliance = false;

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

        myTelem.addData("Current X", currentX);
        myTelem.addData("Current Y", currentY);
        myTelem.addData("Current Theta", currentTheta);

        myTelem.addData("Shooter Velocity: ", shooter.getVelocity());
        myTelem.addData("Shooter Target Velocity: ", shooterDesiredVelocity);
        myTelem.addData("Turret Angle: ", shooter.getTurretPos() / 8.133333333);
        myTelem.addData("Auto State: ", autoState);
        myTelem.addData("Shooter State: ", shootingState);
        myTelem.update();
    }
    private void updateAutoStateMachine() {
        switch (autoState) {


            case PATH_ACTIVE_WAIT:
                if (!follower.isBusy()) {
                    autoTimer.reset();
                    autoState = P_NextState;
                }
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
                autoState = AUTO_STATE.startToShoot;
                turretState = TURRET_STATE.AIMING_NO_TAG;
                break;


            case startToShoot:
                shooter.setMotorVelocity(shooterDesiredVelocity);
                follower.setMaxPower(.86);

                follower.followPath(paths.startToShoot);
                P_NextState = AUTO_STATE.SHOOT;
                S_NextState = AUTO_STATE.shootToIntake;
                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;


            case shootToIntake:
                follower.setMaxPower(.7);
                intake.setPower(1);
                follower.followPath(paths.shootToIntake);
                P_NextState = AUTO_STATE.intake1ToGate;
                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;


            case intake1ToGate:
                follower.setMaxPower(.67);
                follower.followPath(paths.intake1ToGate);
                P_NextState = AUTO_STATE.gateToShoot;
                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;


            case gateToShoot:
                if (autoTimer.time() > 500) {
                    follower.setMaxPower(.9);
                    follower.followPath(paths.gateToShoot);
                    P_NextState = AUTO_STATE.SHOOT;
                    S_NextState = AUTO_STATE.shootToIntake2;
                    autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                }
                break;


            case shootToIntake2:
                follower.setMaxPower(.8);
                intake.setPower(1);
                follower.followPath(paths.shootToIntake2);
                P_NextState = AUTO_STATE.intake2Finish;
                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;


            case intake2Finish:
                follower.setMaxPower(.6);
                follower.followPath(paths.intake2Finish);
                P_NextState = AUTO_STATE.intake2ToShoot;
                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;


            case intake2ToShoot:
                follower.setMaxPower(.8);
                follower.followPath(paths.intake2ToShoot);
                P_NextState = AUTO_STATE.SHOOT;
                S_NextState = AUTO_STATE.leave;
                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;


            case leave:
                follower.followPath(paths.leave);
                shooter.setTurretTarget(0);
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
        if (range < 95)
            velocity = (int) ((0.0586009 * Math.pow(range, 2)) + (-4.2766 * range) + 1498.02814 + 30);
        else
            velocity = (int) ((0.0227675 * Math.pow(range, 2)) + (1.62765 * range) + 1344.59538 + 30);


        shooterDesiredVelocity = Math.min(velocity, 1930);
    }


    private boolean desiredVelocityReached() {
        return shooter.getVelocity() > shooterDesiredVelocity * .97;
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
//                if (innerColor.getDistance(DistanceUnit.INCH) > 4.25)
//                    shootingState = SHOOTING_STATE.END;
                if (shooterTimer.time() > 2000)
                    shootingState = SHOOTING_STATE.END;
                if (desiredVelocityReached())
                    shootingState = SHOOTING_STATE.SHOOT_BALL;
                break;


            case SHOOT_BALL:
                transfer.openTransferGate();
                transfer.setPower(0.87);
                if (shooterTimer.time() > 2000)
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
        turretLastAutoPos = shooter.getTurretPos();
        double turretAngleDeg = shooter.getTurretPos() / 8.13333333333;


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
        public PathChain startToShoot;
        public PathChain shootToIntake;
        public PathChain intake1ToGate;
        public PathChain gateToShoot;
        public PathChain shootToIntake2;
        public PathChain intake2Finish;
        public PathChain intake2ToShoot;
        public PathChain leave;

        public Paths(Follower follower) {
            startToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.177, 119.039),

                                    new Pose(85.553, 84.676)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                    .build();

            shootToIntake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.553, 84.676),

                                    new Pose(125.228, 83.515)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            intake1ToGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(125.228, 83.515),
                                    new Pose(106.362, 72.081),
                                    new Pose(128.233, 72.828)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            gateToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.233, 72.828),

                                    new Pose(85.125, 85.133)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            shootToIntake2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(85.125, 85.133),
                                    new Pose(91.385, 63.254),
                                    new Pose(101.751, 59.738)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            intake2Finish = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(101.751, 59.738),

                                    new Pose(125.733, 58.085)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            intake2ToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(125.733, 58.085),
                                    new Pose(98.154, 59.664),
                                    new Pose(85.294, 83.801)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.294, 83.801),

                                    new Pose(85.885, 61.002)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                    .build();
        }
    }
}

