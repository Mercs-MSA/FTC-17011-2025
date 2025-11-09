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

import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.SoftElectronics;
import org.firstinspires.ftc.teamcode.Teleop;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Spindex;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "PPV Bezier Blue Auto", group = "Autonomous")
@Configurable // Panels
public class PPVBezierBlueAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Paths paths; // Paths defined in the Paths class
    private SoftElectronics softElectronics;

    private Spindex spindex;
    private Intake intake;
    private Shooter shooter;
    private ElapsedTime shootTimer;
    private int timesShot;
    private static Teleop.SHOOTER_STATE rapidFireState = Teleop.SHOOTER_STATE.INACTIVE_STATE;
    private enum AUTO_STATE {
        INIT,
        PATH_ACTIVE_WAIT,
        StartToShootFar,
        IntakeLevel1Ball1,
        IntakeLevel1Ball2,
        IntakeLevel1Ball3,
        IntakeLevel1ToShootFar,
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
        WAIT_UNTIL_SHOOT_DONE
    }

    private AUTO_STATE PAW_NextState; //Path Active Wait Next State
    private AUTO_STATE FR_NextState; //Full Rotate Spindex Next State
    private AUTO_STATE HR_NextState; //Half Rotate Spindex Next State
    private AUTO_STATE S_NextState; //Full Rotate Spindex Next State


    private AUTO_STATE autoState;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        softElectronics = new SoftElectronics(hardwareMap, this.telemetry);

        spindex = new Spindex(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shootTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        autoState = AUTO_STATE.INIT;
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autoState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Auto State", autoState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain StartToShootFar;
        public PathChain IntakeLevel1Ball1;
        public PathChain IntakeLevel1Ball2;
        public PathChain IntakeLevel1Ball3;
        public PathChain IntakeLevel1ToShootFar;
        public PathChain intakeLevel2Ball1;
        public PathChain intakeLevel2Ball2;
        public PathChain intakeLevel2Ball3;
        public PathChain intakeLevel2ToShootClose;
        public PathChain intakeLevel3Ball1;
        public PathChain intakeLevel3Ball2;
        public PathChain intakeLevel3Ball3;
        public PathChain intakeLevel3ToShootClose;
        public PathChain parkByGate;

        public Paths(Follower follower) {
            StartToShootFar = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 8.000),
                                    new Pose(55.790, 17.938),
                                    new Pose(59.739, 21.723)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                    .build();

            IntakeLevel1Ball1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(59.739, 21.723),
                                    new Pose(65.993, 38.016),
                                    new Pose(35.054, 35.383)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                    .build();

            IntakeLevel1Ball2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(35.054, 35.383), new Pose(29.952, 35.383))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            IntakeLevel1Ball3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(29.952, 35.383), new Pose(24.500, 35.383))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            IntakeLevel1ToShootFar = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(24.500, 35.383),
                                    new Pose(40.155, 23.698),
                                    new Pose(59.739, 21.723)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                    .build();

            intakeLevel2Ball1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(59.739, 21.723),
                                    new Pose(66.651, 61.221),
                                    new Pose(34.725, 59.739)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                    .build();

            intakeLevel2Ball2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(34.725, 59.739), new Pose(29.787, 59.739))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            intakeLevel2Ball3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(29.787, 59.739), new Pose(24.192, 59.739))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            intakeLevel2ToShootClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(24.192, 59.739),
                                    new Pose(59.739, 61.056),
                                    new Pose(59.081, 84.425)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            intakeLevel3Ball1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.081, 84.425), new Pose(34.889, 83.767))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();

            intakeLevel3Ball2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(34.889, 83.767), new Pose(29.787, 83.767))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            intakeLevel3Ball3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(29.787, 83.767), new Pose(24.686, 83.767))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            intakeLevel3ToShootClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(24.686, 83.767),
                                    new Pose(43.941, 70.272),
                                    new Pose(59.081, 84.425)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            parkByGate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(59.081, 84.425),
                                    new Pose(49.701, 70.272),
                                    new Pose(20.736, 70.766)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(270))
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

    public AUTO_STATE autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine

        switch (autoState) {
            // General Cases
            case PATH_ACTIVE_WAIT:
                if (!follower.isBusy()) {
                    autoState = PAW_NextState;
                }

                break;

            case FULL_ROTATE_SPINDEX:
                spindex.changeCurrentPositionBy(spindexThirdRevolution);

                autoState = FR_NextState;
                break;

            case HALF_ROTATE_SPINDEX:
                spindex.changeCurrentPositionBy(spindexThirdRevolution/2);

                autoState = HR_NextState;
                break;

            case SHOOT:
                rapidFireState = Teleop.SHOOTER_STATE.START_STATE;

                autoState = AUTO_STATE.WAIT_UNTIL_SHOOT_DONE;
                break;

            case WAIT_UNTIL_SHOOT_DONE:
                if (rapidFireState.equals(Teleop.SHOOTER_STATE.INACTIVE_STATE))
                    autoState = S_NextState;

                break;

            // Ordered Cases
            case StartToShootFar:
                follower.followPath(paths.StartToShootFar);

                PAW_NextState = AUTO_STATE.SHOOT;
                S_NextState = AUTO_STATE.HALF_ROTATE_SPINDEX;
                HR_NextState = AUTO_STATE.IntakeLevel1Ball1;

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                break;

            case IntakeLevel1Ball1:
                intake.setPower(1);
                follower.followPath(paths.IntakeLevel1Ball1);

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                PAW_NextState = AUTO_STATE.IntakeLevel1Ball2;
                break;

            case IntakeLevel1Ball2:
                follower.followPath(paths.IntakeLevel1Ball2);

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                PAW_NextState = AUTO_STATE.IntakeLevel1Ball3;
                break;

            case IntakeLevel1Ball3:
                follower.followPath(paths.IntakeLevel1Ball3);

                autoState = AUTO_STATE.PATH_ACTIVE_WAIT;
                PAW_NextState = AUTO_STATE.IntakeLevel1ToShootFar;
                break;
        }

        return autoState;
    }
}
