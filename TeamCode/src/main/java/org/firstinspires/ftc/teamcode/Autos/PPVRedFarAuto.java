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

import org.firstinspires.ftc.teamcode.Teleop;
import org.firstinspires.ftc.teamcode.mechanisms.Drivebase;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "PPV Red Far Auto", group = "Autonomous")
@Configurable // Panels
public class PPVRedFarAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Drivebase drivebase;
    private Transfer transfer;
    private Intake intake;
    private Shooter shooter;
    private double lastVelocity;
    private int timesShot = 0;
    private double shooterVelocityDropThreshold = 80.6741;

    private enum AUTO_STATE {
        startToShootFar,
        shootToIntakeLevel1,
        intakeLevel1ToShoot,
        shootToTurnToHuman,
        turnToHumanToIntakeHuman,
        intakeHumanToShoot,
        INIT,
        PATH_ACTIVE_WAIT,
        SHOOT,
        WAIT_UNTIL_SHOOT_DONE,
        WAIT_UNTIL_WAIT_DONE,
        WAIT,
        LEAVE,
        END
    }

    private enum SHOOTING_STATE {
        INACTIVE,
        START,
        START_SHOOTER,
        IS_SHOOTER_READY,
        CLOSE_GATE,
        SPIN_UP,
        OPEN_GATE,
        SHOOT,
        END
    }
    public static SHOOTING_STATE shootingState = SHOOTING_STATE.INACTIVE;

    private AUTO_STATE PAW_NextState; //Path Active Wait Next State
    private AUTO_STATE S_NextState; //Full Rotate Spindex Next State
    private AUTO_STATE W_NextState; //Full Rotate Spindex Next State
    private double W_NextStateLengthMS = 670.0;
    private AUTO_STATE autoState;
    private Paths paths; // Paths defined in the Paths class

    private double shooterDesiredVelocity = org.firstinspires.ftc.teamcode.Constants.Constants.FAR_SHOT_VELOCITY;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        drivebase = new Drivebase(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        lastVelocity = shooterDesiredVelocity;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        updatePaths(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Auto State", autoState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain startToShootFar;
        public PathChain shootToIntakeLevel1;
        public PathChain intakeLevel1ToShoot;
        public PathChain shootToTurnToHuman;
        public PathChain turnToHumanToIntakeHuman;
        public PathChain intakeHumanToShoot;

        public Paths(Follower follower) {
            startToShootFar = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.000, 8.000),
                                    new Pose(88.210, 17.938),
                                    new Pose(84.261, 21.723)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))
                    .build();

            shootToIntakeLevel1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(84.261, 21.723),
                                    new Pose(85.054, 33.003),
                                    new Pose(120.192, 35.302)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))
                    .build();

            intakeLevel1ToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120.192, 35.302), new Pose(84.261, 21.723))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))
                    .build();

            shootToTurnToHuman = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(84.261, 21.723), new Pose(84.261, 21.723))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))
                    .build();

            turnToHumanToIntakeHuman = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(84.261, 21.723),
                                    new Pose(100.160, 6.404),
                                    new Pose(134.641, 8.867)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intakeHumanToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(134.641, 8.867), new Pose(84.261, 21.723))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))
                    .build();
        }
    }

    private boolean desiredVelocityReached() {
        return (shooter.getVelocity() > shooterDesiredVelocity * .98);
    }

    public void updatePaths() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    }
    private void shootingMachine() {
        switch (shootingState) {
            case START:
                shootingState = SHOOTING_STATE.SPIN_UP;
                break;

            case START_SHOOTER:
                timesShot = 0;
                shooter.setMotorVelocity(shooterDesiredVelocity);
                transfer.closeTransferGate();

                shootingState = SHOOTING_STATE.IS_SHOOTER_READY;
                break;

            case IS_SHOOTER_READY:
                if (desiredVelocityReached()) {
                    shootingState = SHOOTING_STATE.SHOOT;
                }
                break;

            case SHOOT:
                transfer.openTransferGate();
                transfer.setPower(.87);
                if (shooter.getVelocity() - lastVelocity > shooterVelocityDropThreshold) {
                    timesShot++;
                    if (timesShot < 3) {
                        shootingState = SHOOTING_STATE.START_SHOOTER;
                    } else {
                        shootingState = SHOOTING_STATE.END;
                    }
                }
                lastVelocity = shooter.getVelocity();
                break;
            case END:
                shooter.setMotorVelocity(300);
                transfer.closeTransferGate();
                transfer.setPower(0);
                shootingState = SHOOTING_STATE.INACTIVE;
                break;
            case INACTIVE:
                transfer.closeTransferGate();
                break;
        }
    }
}
