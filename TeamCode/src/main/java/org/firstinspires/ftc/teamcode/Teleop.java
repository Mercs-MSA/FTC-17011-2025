package org.firstinspires.ftc.teamcode;


/*
    Control Hub:
    Motors
    0 - frontLeft
    1 - shooterMotorLeft
    2 - spindexMotor
    3 - backLeft

    Servos

    I2C
    2 - spindexColorB


    Expansion Hub:
    Motors
    0 - frontRight
    1 - backRight
    2 - intakeMotor
    3 - shooterMotorRight

    Servos
    0 - spindexTransferServo

    I2C
    0 - otos

    External Ethernet - limelight
 */

import static org.firstinspires.ftc.teamcode.Constants.Constants.onBlueAlliance;
import static org.firstinspires.ftc.teamcode.Constants.Constants.ranAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Drivebase;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Spindex;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class Teleop extends OpMode {
    public FtcDashboard dash;
    private SoftElectronics softElectronics;
    private Drivebase drivebase;
    private Spindex spindex;
    private Intake intake;
    private Shooter shooter;

    private double drive = 0; // forward/back
    private double strafe = 0; // left/right
    private double turn = 0;  // rotation

    private ElapsedTime rapidFireTimer;

    private static Telemetry myTelem;
    private static TelemetryManager myPanels;

    private double intakePower = 0.0;

    private boolean autoSpun = false;

    public static int farZoneVelocity = 1667;
    public static int closeZoneVelocity = 1450;

    public static int shooterDesiredVelocity = 1667; //1450 for close triangle's end

    public static int spindexFullRevolution = -540; //Amount of encoder positions for one full revolution of spindex
    public static int spindexThirdRevolution = spindexFullRevolution/3; //Amount of encoder positions for one full revolution of spindex

    public enum STARTING_ORIENTATION {
        GOAL_SIDE,
        PLAYER_SIDE
    }

    public static STARTING_ORIENTATION startingOrientation = STARTING_ORIENTATION.GOAL_SIDE;

    public enum SHOOTER_STATE {START_STATE, POINT_AT_GOAL_STATE, RUN_SHOOTER_MOTOR_STATE, WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE, CLOSE_GATE_STATE, RUN_TRANSFER_STATE, RUN_SPINDEX_STATE, RELEASE_STATE, INACTIVE_STATE, END_STATE}
    private static SHOOTER_STATE rapidFireState = SHOOTER_STATE.INACTIVE_STATE;
    private static SHOOTER_STATE motifRapidFireState = SHOOTER_STATE.INACTIVE_STATE;
    public static boolean spinningToColor = false;

    ElapsedTime shotTimer = new ElapsedTime();
    double ema = 0;                             // exponential moving average of RPM
    final double ALPHA = 0.2;                   // smoothing factor (0..1)
    final double DROP_PCT = 0.18;               // 18% dip counts as “ball contact”
    final double RECOVER_PCT = 0.10;            // must recover within 10% of target to re-arm
    boolean shotArmed = true;
    boolean shotDetected = false;

    boolean spindexRunPosition = false;

    @Override
    public void init() {
        // Initialize SoftElectronics
        softElectronics = new SoftElectronics(hardwareMap, this.telemetry);
        dash = FtcDashboard.getInstance();
        myTelem = new MultipleTelemetry(dash.getTelemetry(), softElectronics.getTelemetry());
        myPanels = softElectronics.getPanelsTelemetry();

        // Initialize Drive base
        drivebase = new Drivebase(hardwareMap);
        spindex = new Spindex(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        rapidFireTimer = new ElapsedTime();

        rapidFireState = SHOOTER_STATE.INACTIVE_STATE;
        myTelem.addData("Status", "Initialized");
        myTelem.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.right_bumper) {
            onBlueAlliance = false;
        } else if (gamepad1.left_bumper) {
            onBlueAlliance = true;
        }

//        if (!ranAuto) {
            if (onBlueAlliance) {
                myTelem.addLine("Blue alliance selected. Press right bumper to select red.");
                if (startingOrientation.equals(STARTING_ORIENTATION.GOAL_SIDE)) {
//                    drivebase.offsetYaw(-90);
                    drivebase.setPosition(new SparkFunOTOS.Pose2D(0, 0, -Math.PI/2));
                } else {
//                    drivebase.offsetYaw(90);
                    drivebase.setPosition(new SparkFunOTOS.Pose2D(0, 0, Math.PI/2));
                }
            } else {
                myTelem.addLine("Red alliance selected. Press left bumper to select blue.");
                if (startingOrientation.equals(STARTING_ORIENTATION.GOAL_SIDE)) {
//                    drivebase.offsetYaw(90);
                    drivebase.setPosition(new SparkFunOTOS.Pose2D(0, 0, Math.PI/2));
                } else {
//                    drivebase.offsetYaw(-90);
                    drivebase.setPosition(new SparkFunOTOS.Pose2D(0, 0, -Math.PI/2));
                }
            }
//        }

        if (gamepad1.dpadUpWasPressed()) {
            if (startingOrientation.equals(STARTING_ORIENTATION.PLAYER_SIDE))
                startingOrientation = STARTING_ORIENTATION.GOAL_SIDE;
            else
                startingOrientation = STARTING_ORIENTATION.PLAYER_SIDE;
        }
    }

    @Override
    public void start() {
        super.start();
        //shooter.setShooterPower(1);
//        spindex.reverseTransfer();
        shooter.setMotorVelocity(0);
        ranAuto = false;
    }

    @Override
    public void loop() {
        updateDrivebase();
        updateSpindex();
        updateMechanisms();
        updateTelemetry();
        drivebase.updateLL();
    }

    private void updateTelemetry() {
        //        myTelem.addData("Robot Yaw:", Math.toDegrees(drivebase.getPosition().h));
        myTelem.addData("Robot Heading:", Math.toDegrees(drivebase.getPosition().h));
        myTelem.addData("Robot Offset:", drivebase.getOffset());
//        myTelem.addData("Spinning to Color?", spinningToColor);
//        myTelem.addData("Num of artifacts in robot:", spindex.getNumOfArtifactsInRobot());
        myTelem.addData("Right Color:", spindex.getColor(spindex.spindexColorRight, true));
//        myTelem.addData("Left Color:", spindex.getColor(spindex.spindexColorLeft, true));
//        myTelem.addData("Back Color:", spindex.getColor(spindex.spindexColorBack, true));
        myTelem.addData("rapid fire state:", rapidFireState.toString());
        myTelem.addData("rapid fire timer:", rapidFireTimer.time(TimeUnit.SECONDS));
        myTelem.addData("shooter velocity:", shooter.getRightVelocity());
        myTelem.addData("spindex velocity:", spindex.getSpindexVelocity());
        myTelem.addData("spindex position:", spindex.spindexMotor.getCurrentPosition());
        //telemetry.addData("result valid?", shooter.getLLResults().isValid());
        //telemetry.addData("pipeline", shooter.getLLStatus().getPipelineIndex());
        //telemetry.addData("TX", shooter.getTX() == null ? "null" : shooter.getTX());
        //telemetry.addData("inRange", shooter.inRange());
        myTelem.addData("entry sensor: ", intake.isBallInIntake());
        myTelem.update();
    }

    private void updateShotDetector() {
        double rpm = shooter.getRpm();
        ema = (ALPHA * rpm) + (1 - ALPHA) * ema;

        // arm when we’re basically at speed
        if (!shotArmed && ema > shooterDesiredVelocity * (1.0 - RECOVER_PCT)) {
            shotArmed = true;
        }

        // detect dip
        if (shotArmed && ema < shooterDesiredVelocity * (1.0 - DROP_PCT)) {
            shotDetected = true;
            shotArmed = false;               // prevent double-count
            shotTimer.reset();
        }

        // optional: clear flag after a short window so you can edge-trigger it
        if (shotDetected && shotTimer.seconds() > 0.25) {
            shotDetected = false;
        }
    }

    private void updateSpindex() {
        if (!spindex.isSpindexMoving() && intake.isBallInIntake()) {
            spindex.changeCurrentPositionBy(spindexThirdRevolution);
        }
    }

    private void updateDrivebase() {
        // Field-centric driving
        drive = -gamepad1.left_stick_y; // forward/back
        strafe = gamepad1.left_stick_x; // left/right
        turn = gamepad1.right_stick_x;  // rotation

        if (rapidFireState != SHOOTER_STATE.INACTIVE_STATE || motifRapidFireState != SHOOTER_STATE.INACTIVE_STATE) {
            drivebase.stop();
        } else {
            drivebase.drive(drive, strafe, turn);
        }

        if (gamepad1.dpad_up) {
//            if (drivebase.getTargetSeen()) gamepad1.rumble(50);
//            drivebase.turnToGoal();
            drivebase.resetYaw();
        }

        if (gamepad1.circle) drivebase.turnToGoal();

        if (gamepad1.right_bumper) {
            drive *= .6;
            strafe *= .6;
            turn *= .6;
        }
    }


    private void updateMechanisms() {
        updateRapidFireStateMachine();
        updateShotDetector();

        if (gamepad2.left_bumper) { //Outake
            intakePower = -1;
        } else if (gamepad2.right_bumper /* && !spindex.checkIfIntaked()*/) { //Intake
            intakePower = 1;
        } else if (rapidFireState.equals(SHOOTER_STATE.INACTIVE_STATE) /*|| spindex.checkIfIntaked()*/) {
            intakePower = 0;
        }

        //Auto Intake
        if (false) {
            intakePower = 1;
            /*if (spindex.checkIfIntaked()) {

            }
             */
        }


        intake.setPower(intakePower);

        if (gamepad1.right_trigger > 0.5 && rapidFireState.equals(SHOOTER_STATE.INACTIVE_STATE)) {
            rapidFireState = SHOOTER_STATE.START_STATE;
        }

        if (gamepad2.right_trigger > .5) {
            spindex.reverseTransfer();
        } else if (!rapidFireState.equals(SHOOTER_STATE.INACTIVE_STATE)) {
        } else {
            spindex.stopTransferWheel();
        }

        if (gamepad1.left_trigger > 0.5) {
            shooterDesiredVelocity = closeZoneVelocity;
        } else
            shooterDesiredVelocity = farZoneVelocity;

        if (gamepad2.crossWasPressed()) {
            spindex.changeCurrentPositionBy(spindexThirdRevolution);
        }

        if (gamepad2.circleWasPressed()) {
            spindex.changeCurrentPositionBy(spindexThirdRevolution/2);
        }

        if (gamepad2.squareWasPressed()) {
            spindex.resetSpindexToZero();
        }

        if (gamepad1.dpadUpWasPressed()) {
            drivebase.resetYaw();
        }

        if (gamepad2.left_trigger > .5) {
            spindex.runTransferWheel();
        }
    }

    private void updateRapidFireStateMachine() {
        switch (rapidFireState) {
            case START_STATE:
                rapidFireState = SHOOTER_STATE.RUN_SHOOTER_MOTOR_STATE;
                break;

            case RUN_SHOOTER_MOTOR_STATE:
                shooter.setMotorVelocity(shooterDesiredVelocity);
                spindex.runSpindexToTransferThird();

                //Go to Next State
                rapidFireState = SHOOTER_STATE.WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE;

                //Cancel State Machine
                if (drive > .1 || drive < -.1 || strafe > .1 || strafe < -.1) {
                    rapidFireState = SHOOTER_STATE.END_STATE;
                }
                break;

            case RUN_SPINDEX_STATE:
                spindex.changeCurrentPositionBy(spindexThirdRevolution);
                rapidFireState = SHOOTER_STATE.WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE;
                break;

            case WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE:

                //Go to next state when Artifact is in position AND shooter has reached desired velocity
                if (shooter.getRightVelocity() > shooterDesiredVelocity * .95 && Math.abs(spindex.spindexMotor.getCurrentPosition() - Spindex.currentSpindexPosition) < 3) {
//                    if (spindex.getColor(spindex.spindexColorBack).equals(GeneralConstants.colorSensorStates.OCCUPIED))
                    rapidFireState = SHOOTER_STATE.RUN_TRANSFER_STATE;
//                    else
//                        rapidFireState = SHOOTER_STATE.RUN_SPINDEX_STATE;
                    rapidFireTimer.reset();
                }

                //Cancel State Machine
                if (drive > .1 || drive < -.1 || strafe > .1 || strafe < -.1) {
                    rapidFireState = SHOOTER_STATE.END_STATE;
                }

                break;

            case RUN_TRANSFER_STATE:
                spindex.runTransferWheel();

                //Repeat RUN_SPINDEX State when timer has reached 3 seconds or when artifact is shot
                if (/*rapidFireTimer.time(TimeUnit.SECONDS) > 3 ||*/ gamepad1.right_trigger > 0.5) {
                    rapidFireTimer.reset();
                    rapidFireState = SHOOTER_STATE.RUN_SPINDEX_STATE;
                }

                //Cancel State Machine
                if (drive > .1 || drive < -.1 || strafe > .1 || strafe < -.1) {
                    rapidFireState = SHOOTER_STATE.END_STATE;
                }

//                if (gamepad1.triangle) rapidFireState = SHOOTER_STATE.END_STATE;

                break;

            case END_STATE:
                shooter.stop();
                spindex.stopTransferWheel();
                spindex.stopSpindex();

                rapidFireState = SHOOTER_STATE.INACTIVE_STATE;

                break;

            case INACTIVE_STATE:
                break;
        }
    }
}


