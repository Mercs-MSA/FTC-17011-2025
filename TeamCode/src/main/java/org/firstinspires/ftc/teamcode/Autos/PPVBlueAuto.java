/*
package org.firstinspires.ftc.teamcode.pedroPathingVisualizerTestPrograms;

import static org.firstinspires.ftc.teamcode.Teleop.spindexThirdRevolution;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.SoftElectronics;
import org.firstinspires.ftc.teamcode.Teleop;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Spindex;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "PPV Blue Auto", group = "Autonomous")
@Configurable // Panels
public class PPVBlueAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private SoftElectronics softElectronics;
    private Spindex spindex;
    private Intake intake;
    private Shooter shooter;
    private ElapsedTime shootTimer;
    private int timesShot;

    private static Teleop.SHOOTER_STATE rapidFireState = Teleop.SHOOTER_STATE.INACTIVE_STATE;

    @Override
    public void init() {

        softElectronics = new SoftElectronics(hardwareMap, this.telemetry);

        spindex = new Spindex(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shootTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine
        updateRapidFireStateMachine();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain toShootFar1;
        public PathChain toStartIntake1;
        public PathChain toEndIntake1;
        public PathChain toShootFar2;
        public PathChain toStartIntake2;
        public PathChain toEndIntake2;
        public PathChain toShootClose1;
        public PathChain toStartIntake3;
        public PathChain toEndIntake3;
        public PathChain toShootClose2;

        public Paths(Follower follower) {
            toShootFar1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(60.300, 22.700))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                    .build();

            toStartIntake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.300, 22.700), new Pose(41.000, 35.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                    .build();

            toEndIntake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(41.000, 35.500), new Pose(22.200, 35.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            toShootFar2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(22.200, 35.500), new Pose(60.300, 22.700))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                    .build();

            toStartIntake2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.300, 22.700), new Pose(41.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                    .build();

            toEndIntake2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(41.000, 60.000), new Pose(22.200, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            toShootClose1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(22.200, 60.000), new Pose(58.000, 85.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            toStartIntake3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.000, 85.000), new Pose(41.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();

            toEndIntake3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(41.000, 84.000), new Pose(22.200, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            toShootClose2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(22.200, 84.000), new Pose(58.000, 85.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();
        }
    }
    private void updateRapidFireStateMachine() {
        switch (rapidFireState) {
            case START_STATE:
                rapidFireState = Teleop.SHOOTER_STATE.RUN_SHOOTER_MOTOR_STATE;
                break;

            case RUN_SHOOTER_MOTOR_STATE:
                shooter.setMotorVelocity(Teleop.shooterDesiredVelocity);
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
                if (shooter.getRightVelocity() > Teleop.shooterDesiredVelocity * .95 && Math.abs(spindex.spindexMotor.getCurrentPosition() - Spindex.currentSpindexPosition) < 3) {
                    if (spindex.getColor(spindex.spindexColorRight).equals(GeneralConstants.colorSensorStates.OCCUPIED))
                        rapidFireState = Teleop.SHOOTER_STATE.RUN_TRANSFER_STATE;
                    else
                        rapidFireState = Teleop.SHOOTER_STATE.RUN_SPINDEX_STATE;
                    shootTimer.reset();
                }
                break;

            case RUN_TRANSFER_STATE:
                spindex.runTransferWheel();

                //Repeat RUN_SPINDEX State when timer has reached 3 seconds or when artifact is shot
                if (shootTimer.time(TimeUnit.SECONDS) > 3 && timesShot < 3) {
                    shootTimer.reset();
                    timesShot++;
                    rapidFireState = Teleop.SHOOTER_STATE.RUN_SPINDEX_STATE;
                } else if (timesShot == 3){
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

    public int autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                follower.followPath(paths.toShootFar1);

                if (!follower.isBusy())
                    pathState++;
                break;

            case 1:
                rapidFireState = Teleop.SHOOTER_STATE.START_STATE;

                if (rapidFireState == Teleop.SHOOTER_STATE.END_STATE)
                    pathState++;
                break;
        }

        return pathState;
    }
}
*/