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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Drivebase;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static org.firstinspires.ftc.teamcode.Constants.Constants.blueGoal;
import static org.firstinspires.ftc.teamcode.Constants.Constants.redGoal;
import static org.firstinspires.ftc.teamcode.Constants.Constants.onBlueAlliance;

@Autonomous(name = "PPV Red Far Auto", group = "Autonomous")
@Configurable
public class PPVRedFarAuto extends OpMode {

    /* ===================== TELEMETRY / PATHING ===================== */

    private TelemetryManager panelsTelemetry;
    public Follower follower;

    /* ===================== MECHANISMS ===================== */

    private Drivebase drivebase;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;

    /* ===================== TIMING ===================== */

    private ElapsedTime autoTimer;
    private double time = 0;

    /* ===================== SHOOTER ===================== */

    private double shooterDesiredVelocity = 0;

    /* ===================== ENUMS ===================== */

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
        AIMING_TO_TAG
    }



    private AUTO_STATE autoState = AUTO_STATE.INIT;
    private AUTO_STATE P_NextState;
    private AUTO_STATE S_NextState;

    public static SHOOTING_STATE shootingState = SHOOTING_STATE.INACTIVE;
    private TURRET_STATE turretState = TURRET_STATE.AIMING_TO_TAG;

    private Paths paths;



    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        drivebase = new Drivebase(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        autoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }



    @Override
    public void loop() {
        time = autoTimer.time();

        follower.update();
        updateAutoStateMachine();
        updateShooterStateMachine();
        updateTurretState();

        panelsTelemetry.update(telemetry);
    }



    private void updateAutoStateMachine() {
        switch (autoState) {

            case PATH_ACTIVE_WAIT:
                if (!follower.isBusy())
                    autoState = P_NextState;
                break;

            case SHOOT:
                if (shootingState == SHOOTING_STATE.INACTIVE &&
                        follower.atPose(
                                new Pose(84.26057142857142, 21.723),
                                2.25, 2.25
                        )) {

                    setShooterDesiredVelocity();
                    autoTimer.reset();
                    shootingState = SHOOTING_STATE.START;
                }
                else if (shootingState == SHOOTING_STATE.END) {
                    shootingState = SHOOTING_STATE.INACTIVE;
                    autoState = S_NextState;
                }
                break;

            case INIT:
                autoState = AUTO_STATE.startToShootFar;
                break;

            case startToShootFar:
                setShooterDesiredVelocity();
                shooter.setMotorVelocity(shooterDesiredVelocity);

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
                follower.followPath(paths.shootToIntakeHuman1);
                P_NextState = AUTO_STATE.intakeHuman1to2;
                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;

            case intakeHuman1to2:
                follower.followPath(paths.intakeHuman1to2);
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



    private void setShooterDesiredVelocity() {
        double range = drivebase.distanceToTarget();
        int velocity = (int) (
                (0.0586009 * Math.pow(range, 2)) +
                        (-4.2766 * range) +
                        1498.02814
        );
        shooterDesiredVelocity = Math.min(velocity, 1940);
    }

    private boolean desiredVelocityReached() {
        return shooter.getVelocity() > shooterDesiredVelocity * 0.97;
    }

    private void updateShooterStateMachine() {
        switch (shootingState) {

            case START:
                shootingState = SHOOTING_STATE.START_SHOOTER;
                break;

            case START_SHOOTER:
                setShooterDesiredVelocity();
                shooter.setMotorVelocity(shooterDesiredVelocity);
                transfer.closeTransferGate();
                shootingState = SHOOTING_STATE.IS_SHOOTER_READY;
                break;

            case IS_SHOOTER_READY:
                if (desiredVelocityReached())
                    shootingState = SHOOTING_STATE.SHOOT_BALL;
                else if (time > 5000)
                    shootingState = SHOOTING_STATE.END;
                break;

            case SHOOT_BALL:
                transfer.openTransferGate();
                transfer.setPower(0.87);
                if (!desiredVelocityReached() || time > 5000)
                    shootingState = SHOOTING_STATE.END;
                break;

            case END:
                shooter.setMotorVelocity(800);
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

        double error = AngleUnit.normalizeDegrees(turretSetpointDeg - turretAngleDeg);

        boolean seesTarget = drivebase.getTargetSeen();
        boolean withinAngleLimit =
                Math.abs(turretAngleDeg) < 110 &&
                        Math.abs(targetFieldAngle) < 110;

        switch (turretState) {

            case GO_TO_ZERO:
                shooter.setTurretVelocity(200, 1);
                shooter.setTurretTarget(0);
                if (Math.abs(shooter.getTurretPos()) < 10)
                    turretState = TURRET_STATE.ZEROED;
                break;

            case ZEROED:
                shooter.setTurretVelocity(0, 0);
                if (seesTarget)
                    turretState = TURRET_STATE.AIMING_TO_TAG;
                break;

            case AIMED:
                shooter.setTurretVelocity(0, 0);
                turretState = seesTarget
                        ? TURRET_STATE.AIMING_TO_TAG
                        : TURRET_STATE.AIMING_NO_TAG;
                break;

            case AIMING_NO_TAG:
                if (seesTarget) {
                    turretState = TURRET_STATE.AIMING_TO_TAG;
                    return;
                }
                if (Math.abs(error) > 1)
                    shooter.setTurretTargetShortestPath(turretSetpointDeg);
                else
                    turretState = TURRET_STATE.AIMED;
                break;

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
        }
    }



    public static class Paths {
        public PathChain startToShootFar;
        public PathChain shootToIntakeLevel1;
        public PathChain intakeLevel1ToShoot;
        public PathChain shootToIntakeHuman1;
        public PathChain intakeHuman1to2;
        public PathChain intakeHumanToShoot;

        public Paths(Follower follower) {

            startToShootFar = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(88, 8),
                            new Pose(88.210, 17.938),
                            new Pose(84.261, 21.723)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))
                    .build();

            shootToIntakeLevel1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(84.261, 21.723),
                            new Pose(89.477, 38.680),
                            new Pose(101.474, 28.000),
                            new Pose(125.455, 35.302)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))
                    .build();

            intakeLevel1ToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(125.455, 35.302),
                            new Pose(84.261, 21.723)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))
                    .build();

            shootToIntakeHuman1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(84.261, 21.723),
                            new Pose(132.581, 29.561),
                            new Pose(132.746, 16.446)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(-30))
                    .build();

            intakeHuman1to2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(132.746, 16.446),
                            new Pose(133.135, 10.769)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-30))
                    .build();

            intakeHumanToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(133.135, 10.769),
                            new Pose(116.872, 22.769),
                            new Pose(84.261, 21.723)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(65))
                    .build();
        }
    }
}
