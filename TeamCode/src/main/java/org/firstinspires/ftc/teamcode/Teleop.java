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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    private ElapsedTime intakeTimer;

    private static Telemetry myTelem;
    private static TelemetryManager myPanels;

    private double intakePower = 0.0;
    public static int farZoneVelocity = 1600;

    public static int closeZoneVelocity = 1250;
    public static double farZoneHeading = 65.0;
    public static double closeZoneHeading = 45.0;

    public static int shooterDesiredVelocity = farZoneVelocity; //1450 for close triangle's end

    //12,905 for 24 full revolutions = 537.70833333
    //16,129 for for 30 full revolutions = 537.633333
    //avg = 537.67 (i cant even make this up)
    public static double spindexFullRevolution = -537.67; //Amount of encoder positions for one full revolution of spindex
    public static double spindexThirdRevolution = spindexFullRevolution/3.0; //Amount of encoder positions for one full revolution of spindex
    private boolean checkAgain = true;
    public static double shooterVelocityDropThreshold = 100.6741;

    public enum STARTING_ORIENTATION {
        GOAL_SIDE,
        PLAYER_SIDE
    }

    public static STARTING_ORIENTATION startingOrientation = STARTING_ORIENTATION.GOAL_SIDE;

    public enum SHOOTER_STATE {START_STATE, POINT_AT_GOAL_STATE, WAIT_UNTIL_ROBOT_TURNED, RUN_SHOOTER_MOTOR_STATE, WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE, RUN_TRANSFER_STATE, RUN_SPINDEX_STATE, INACTIVE_STATE, END_STATE}
    public enum INTAKE_STATE {EMPTY, JUST_INTOOK_BALL, CYCLING_BALL}

    public enum AUTO_AIM_STATE {INACTIVE, AIMING_NO_TAG, AIMING_WITH_TAG, WAIT_FOR_ROBOT_TO_FINISH_TURNING, END,}

    public enum MOTIF_PATTERN {GPP, PGP, PPG}

    private MOTIF_PATTERN motifPattern = MOTIF_PATTERN.GPP;

    private static int numOfBallsInRobot = 0;
    private static INTAKE_STATE intakeState = INTAKE_STATE.EMPTY;
    private static SHOOTER_STATE shooterState = SHOOTER_STATE.INACTIVE_STATE;
    private SHOOTER_STATE motifShooterState = SHOOTER_STATE.INACTIVE_STATE;
    private AUTO_AIM_STATE autoAimState = AUTO_AIM_STATE.INACTIVE;
    private double AA_targetHeading;

    public static int motifBallNumber = 1;

    private static double lastShooterVelocity;

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
        intakeTimer = new ElapsedTime();

        lastShooterVelocity = shooter.getRightVelocity();

        shooterState = SHOOTER_STATE.INACTIVE_STATE;
        intakeState = INTAKE_STATE.EMPTY;
        numOfBallsInRobot = 0;
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
                    drivebase.offsetYaw(-90);
//                    drivebase.setPosition(new SparkFunOTOS.Pose2D(0, 0, -Math.PI/2));
                } else {
                    drivebase.offsetYaw(90);
//                    drivebase.setPosition(new SparkFunOTOS.Pose2D(0, 0, Math.PI/2));
                }
            } else {
                myTelem.addLine("Red alliance selected. Press left bumper to select blue.");
                if (startingOrientation.equals(STARTING_ORIENTATION.GOAL_SIDE)) {
                    drivebase.offsetYaw(90);
//                    drivebase.setPosition(new SparkFunOTOS.Pose2D(0, 0, Math.PI/2));
                } else {
                    drivebase.offsetYaw(-90);
//                    drivebase.setPosition(new SparkFunOTOS.Pose2D(0, 0, -Math.PI/2));
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

        if (onBlueAlliance) {
//            drivebase.setPosition(new SparkFunOTOS.Pose2D(136, 8, Math.toRadians(180)));
            farZoneHeading = -70.0;
            closeZoneHeading = 45.0;
        } else {
//            drivebase.setPosition(new SparkFunOTOS.Pose2D(8, 8, Math.toRadians(0)));
            farZoneHeading = 67.0;
            closeZoneHeading = -45.0;
        }
    }

    @Override
    public void loop() {

        updateDrivebase();
        updateMechanisms();
        updateSpindexAndIntakeStateMahcine();


        updateAutoAimStateMachine();
        updateShooterStateMachine();

        updateMotifShooterStateMachine();
        updateTelemetry();

        drivebase.updateLL();
    }

    private void updateTelemetry() {
        //        myTelem.addData("Robot Yaw:", Math.toDegrees(drivebase.getPosition().h));
        myTelem.addData("Robot Heading:", Math.toDegrees(drivebase.getPosition().h));
        myTelem.addData("Heading Error (Far)", AngleUnit.DEGREES.normalize((farZoneHeading - AngleUnit.DEGREES.normalize(Math.toDegrees(drivebase.getPosition().h))) * -1));
        //myTelem.addData("Robot Offset:", drivebase.getOffset());
        //myTelem.addData("Right Color:", spindex.getColor(spindex.spindexColorRight, true));
        myTelem.addData("rapid fire state:", shooterState.toString());
        myTelem.addData("rapid fire timer:", rapidFireTimer.time(TimeUnit.SECONDS));
        myTelem.addData("shooter velocity:", shooter.getRightVelocity());
        myTelem.addLine("\n");

        myTelem.addData("spindex actual position:", spindex.spindexMotor.getCurrentPosition());
        myTelem.addData("spindex desired position:", Spindex.currentSpindexPosition);
        myTelem.addData("entry sensor: ", intake.isBallInIntake());
        myTelem.addLine("\n");

        myTelem.addData("intake timer:", intakeTimer.time(TimeUnit.SECONDS));
        myTelem.addData("intake/spindex state: ", intakeState.toString());
        myTelem.addData("numOfBalls: ", numOfBallsInRobot);
        myTelem.addLine("\n");

        myTelem.addData("Motif Pattern: ", motifPattern.toString());
        myTelem.addData("Motif Current Ball: ", motifBallNumber);
        myTelem.addData("Motif Shooter State: ", motifShooterState.toString());
        myTelem.addLine("\n");

        myTelem.addData("autoAimState: ", autoAimState.toString());

        myTelem.addData("Left Color Sensor", spindex.getColorRaw(spindex.spindexColorLeft));
        myTelem.addData("Right Color Sensor", spindex.getColorRaw(spindex.spindexColorRight));
        myTelem.addData("Back Color Sensor", spindex.getColorRaw(spindex.spindexColorBack));
        myTelem.addLine("\n");

        myTelem.addData("Left Color Sensor", spindex.getColor(spindex.spindexColorLeft));
        myTelem.addData("Right Color Sensor", spindex.getColor(spindex.spindexColorRight));
        myTelem.addData("Back Color Sensor", spindex.getColor(spindex.spindexColorBack));
        myTelem.addLine("\n");

        myTelem.update();
    }





    private void updateDrivebase() {
        // Field-centric driving
        drive = -gamepad1.left_stick_y; // forward/back
        strafe = gamepad1.left_stick_x; // left/right
        turn = gamepad1.right_stick_x;  // rotation

        if (shooterState == SHOOTER_STATE.INACTIVE_STATE && autoAimState == AUTO_AIM_STATE.INACTIVE) {
            drivebase.drive(drive, strafe, turn);
        }

        if (gamepad1.triangle) autoAimState = AUTO_AIM_STATE.AIMING_NO_TAG;
                else autoAimState = AUTO_AIM_STATE.INACTIVE;


        if (gamepad1.left_bumper) {
            drive *= .6;
            strafe *= .6;
            turn *= .6;
        }
    }


    private void updateMechanisms() {
        if (gamepad1.left_bumper) { //Outake
            intakePower = -1;
        } else if (gamepad1.right_bumper /* && !spindex.checkIfIntaked()*/) { //Intake
            intakePower = 1;
        } else if (shooterState.equals(SHOOTER_STATE.INACTIVE_STATE) /*|| spindex.checkIfIntaked()*/) {
            intakePower = 0;
        }


        intake.setPower(intakePower);

        if (gamepad1.right_trigger > 0.5 && shooterState.equals(SHOOTER_STATE.INACTIVE_STATE)) {
            shooterState = SHOOTER_STATE.START_STATE;
        }

        if (gamepad1.left_trigger > 0.5) {
            shooterDesiredVelocity = closeZoneVelocity;
        } else
            shooterDesiredVelocity = farZoneVelocity;

        if (gamepad1.crossWasPressed()) {
            intakeState = INTAKE_STATE.JUST_INTOOK_BALL;
            checkAgain = true;
        }

        if (gamepad1.circleWasPressed()) {
            spindex.changeCurrentPositionBy(spindexThirdRevolution/2.0);
        }

        if (gamepad1.squareWasPressed()) {
            numOfBallsInRobot = 0;
        }

        if (gamepad1.dpadUpWasPressed()) {
            drivebase.resetYaw();
        }

        if (gamepad1.dpadRightWasPressed()) {
            if (motifPattern.equals(MOTIF_PATTERN.GPP)) {
                motifPattern = MOTIF_PATTERN.PGP;
            } else if (motifPattern.equals(MOTIF_PATTERN.PGP)) {
                motifPattern = MOTIF_PATTERN.PPG;
            } else if (motifPattern.equals(MOTIF_PATTERN.PPG)) {
                motifPattern = MOTIF_PATTERN.GPP;
            }
        }

        if (gamepad1.dpadLeftWasPressed()) {
            if (motifPattern.equals(MOTIF_PATTERN.PPG)) {
                motifPattern = MOTIF_PATTERN.PGP;
            } else if (motifPattern.equals(MOTIF_PATTERN.GPP)) {
                motifPattern = MOTIF_PATTERN.PPG;
            } else if (motifPattern.equals(MOTIF_PATTERN.PGP)) {
                motifPattern = MOTIF_PATTERN.GPP;
            }
        }

    }

    private void updateMotifShooterStateMachine() {


        switch (motifShooterState) {
            case START_STATE:
                motifShooterState = SHOOTER_STATE.RUN_SHOOTER_MOTOR_STATE;
                break;

            case RUN_SHOOTER_MOTOR_STATE:
                shooter.setMotorVelocity(shooterDesiredVelocity);
                spindex.changeCurrentPositionBy(spindexThirdRevolution/2.0);
                motifBallNumber = 1;

                //Go to Next State
                motifShooterState = SHOOTER_STATE.WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE;

                //Cancel State Machine
                if (drive > .1 || drive < -.1 || strafe > .1 || strafe < -.1) {
                    motifShooterState = SHOOTER_STATE.END_STATE;
                }
                break;

            case RUN_SPINDEX_STATE:
                //back, right, left,
                String[] artifactArray = {
                        spindex.getColor(spindex.spindexColorBack),
                        spindex.getColor(spindex.spindexColorRight),
                        spindex.getColor(spindex.spindexColorLeft),


                };

                String desiredBallColor = motifPattern.toString().substring(motifBallNumber-1, motifBallNumber);

                if (artifactArray[0].equals(desiredBallColor)) {
                    motifShooterState = SHOOTER_STATE.WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE;
                    break;
                }

                if (artifactArray[1].equals(desiredBallColor)) {
                    spindex.changeCurrentPositionBy(spindexThirdRevolution);
                    motifShooterState = SHOOTER_STATE.WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE;
                    break;
                }

                if (artifactArray[2].equals(desiredBallColor)) {
                    spindex.changeCurrentPositionBy(-spindexThirdRevolution);
                    motifShooterState = SHOOTER_STATE.WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE;
                    break;
                }


                if (!(artifactArray[0].equals("EMPTY")) ) {
                    motifShooterState = SHOOTER_STATE.WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE;
                    break;
                }

                if (!(artifactArray[1].equals("EMPTY")) ) {
                    spindex.changeCurrentPositionBy(spindexThirdRevolution);
                    motifShooterState = SHOOTER_STATE.WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE;
                    break;
                }

                if (!(artifactArray[2].equals("EMPTY")) ) {
                    spindex.changeCurrentPositionBy(-spindexThirdRevolution);
                    motifShooterState = SHOOTER_STATE.WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE;
                    break;
                }

                motifShooterState = SHOOTER_STATE.END_STATE;

                break;

            case WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE:

                //Go to next state when Artifact is in position AND shooter has reached desired velocity
                if (shooter.getRightVelocity() > shooterDesiredVelocity * .97 && !spindex.isSpindexMoving()) {
                    motifShooterState = SHOOTER_STATE.RUN_TRANSFER_STATE;
                    rapidFireTimer.reset();
                }

                //Cancel State Machine
                if (drive > .1 || drive < -.1 || strafe > .1 || strafe < -.1) {
                    motifShooterState = SHOOTER_STATE.END_STATE;
                }

                break;

            case RUN_TRANSFER_STATE:
                spindex.runTransferWheel();

                //Repeat RUN_SPINDEX State when timer has reached 3 seconds or when artifact is shot
                if (lastShooterVelocity - shooter.getRightVelocity() > shooterVelocityDropThreshold || gamepad1.right_trigger > 0.5) {
                    rapidFireTimer.reset();
                    numOfBallsInRobot--;
                    if (numOfBallsInRobot < 0)
                        numOfBallsInRobot = 0;

                    motifBallNumber++;
                    motifShooterState = SHOOTER_STATE.RUN_SPINDEX_STATE;
                }

                lastShooterVelocity = shooter.getRightVelocity();

                //Cancel State Machine
                if (drive > .1 || drive < -.1 || strafe > .1 || strafe < -.1) {
                    motifShooterState = SHOOTER_STATE.END_STATE;
                }

                break;

            case END_STATE:
                shooter.stop();
                spindex.stopTransferWheel();
                spindex.changeCurrentPositionBy(spindexThirdRevolution/2.0);

                motifShooterState = SHOOTER_STATE.INACTIVE_STATE;

                break;

            case INACTIVE_STATE:
                break;
        }
    }

    private void updateShooterStateMachine() {

        switch (shooterState) {
            case START_STATE:
                spindex.changeCurrentPositionBy(spindexThirdRevolution/2.0);
                shooterState = SHOOTER_STATE.RUN_SHOOTER_MOTOR_STATE;
                break;

            case RUN_SHOOTER_MOTOR_STATE:
                shooter.setMotorVelocity(shooterDesiredVelocity);

                //Go to Next State
                shooterState = SHOOTER_STATE.RUN_SPINDEX_STATE;

                //Cancel State Machine
                if (drive > .1 || drive < -.1 || strafe > .1 || strafe < -.1) {
                    shooterState = SHOOTER_STATE.END_STATE;
                }
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
//                double headingError = currentHeading - (shooterDesiredVelocity == farZoneVelocity ? farZoneHeading : closeZoneHeading) ;
//
//                if (Math.abs(headingError) < 2.0) {   // robot is basically facing the target
//                    drivebase.stop();  // stop motors
//                    shooterState = SHOOTER_STATE.RUN_SPINDEX_STATE;
//                }
//
//                drivebase.setDrivePower(turnPower, -turnPower, turnPower, -turnPower);


                break;

            case RUN_SPINDEX_STATE:
                spindex.changeCurrentPositionBy(spindexThirdRevolution);
                shooterState = SHOOTER_STATE.WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE;
                break;

            case WAIT_UNTIL_SHOOTER_SPINDEX_READY_STATE:

                if (shooter.getRightVelocity() > shooterDesiredVelocity * .97 && !spindex.isSpindexMoving()) {
                    shooterState = SHOOTER_STATE.RUN_TRANSFER_STATE;
                }

                //Cancel State Machine
                if (drive > .1 || drive < -.1 || strafe > .1 || strafe < -.1) {
                    shooterState = SHOOTER_STATE.END_STATE;
                }

                break;

            case RUN_TRANSFER_STATE:
                spindex.runTransferWheel();

                //Repeat RUN_SPINDEX State when timer has reached 3 seconds or when artifact is shot
                if (lastShooterVelocity - shooter.getRightVelocity() > shooterVelocityDropThreshold || gamepad1.right_trigger > 0.5) {
                    rapidFireTimer.reset();
                    numOfBallsInRobot--;
                    if (numOfBallsInRobot < 0)
                        numOfBallsInRobot = 0;
                    shooterState = SHOOTER_STATE.RUN_SPINDEX_STATE;
                }

                lastShooterVelocity = shooter.getRightVelocity();

                //Cancel State Machine
                if (drive > .1 || drive < -.1 || strafe > .1 || strafe < -.1) {
                    shooterState = SHOOTER_STATE.END_STATE;
                }

                break;

            case END_STATE:
                shooter.stop();
                spindex.stopTransferWheel();
                spindex.changeCurrentPositionBy(spindexThirdRevolution/2.0);

                shooterState = SHOOTER_STATE.INACTIVE_STATE;

                break;

            case INACTIVE_STATE:
                break;
        }
    }

    private void updateAutoAimStateMachine() {

        switch (autoAimState) {

            case INACTIVE:
                break;

            case AIMING_NO_TAG:
                // Pick target depending on alliance
//                if (onBlueAlliance) {
//                    AA_targetHeading = Drivebase.getPointsHeading(
//                            Drivebase.blueAimPointX,
//                            Drivebase.blueAimPointy,
//                            Drivebase.otos.getPosition().x,
//                            Drivebase.otos.getPosition().y
//                    );

//                } else {
//                    AA_targetHeading = Drivebase.getPointsHeading(
//                            Drivebase.redAimPointX,
//                            Drivebase.redAimPointy,
//                            Drivebase.otos.getPosition().x,
//                            Drivebase.otos.getPosition().y
//                    );
//                }
//                if (drivebase.getResults() != null) {
//                    autoAimState = AUTO_AIM_STATE.AIMING_WITH_TAG;
//                }

                double goalHeading = shooterDesiredVelocity == farZoneVelocity ? farZoneHeading : closeZoneHeading;


                double botHeading = Math.toDegrees(drivebase.getPosition().h);

                // You can change this tolerance depending on how crispy you want aim to be
                double headingError = Math.abs(AngleUnit.DEGREES.normalize(botHeading - goalHeading));

                if (headingError < 2.0) {   // robot is basically facing the target
                    drivebase.stop();  // stop motors
                    autoAimState = AUTO_AIM_STATE.END;
                } else {
                    drivebase.turnToHeading(goalHeading);
                }

//                autoAimState = AUTO_AIM_STATE.WAIT_FOR_ROBOT_TO_FINISH_TURNING;
                break;


            case WAIT_FOR_ROBOT_TO_FINISH_TURNING:
//                double botHeading = Math.toDegrees(drivebase.getPosition().h);
//
//                // You can change this tolerance depending on how crispy you want aim to be
//                double headingError = Math.abs(botHeading - farZoneHeading);
//
//                if (headingError < 2.0) {   // robot is basically facing the target
//                    drivebase.stop();  // stop motors
//                    autoAimState = AUTO_AIM_STATE.END;
//                }

                break;


            case END:
                autoAimState = AUTO_AIM_STATE.INACTIVE;
                break;
        }
    }


    private void updateSpindexAndIntakeStateMahcine() {
        switch (intakeState) {
            case EMPTY:
                if (!spindex.isSpindexMoving() && intake.isBallInIntake() && numOfBallsInRobot < 3 && shooterState.equals(SHOOTER_STATE.INACTIVE_STATE) && motifShooterState.equals(SHOOTER_STATE.INACTIVE_STATE)) {
                    intakeTimer.reset();
                    intakeState = INTAKE_STATE.CYCLING_BALL; //SWITCH
                }
                break;

            case JUST_INTOOK_BALL:
                if (intakeTimer.time(TimeUnit.SECONDS) > 0.05 || checkAgain) {
                    if (checkAgain) {
                        intakeState = INTAKE_STATE.CYCLING_BALL;
                        checkAgain = false;
                    } else {
                        intakeState = INTAKE_STATE.EMPTY;
                        checkAgain = true;
                    }
                }
                break;

            case CYCLING_BALL:
                numOfBallsInRobot++;

                if (numOfBallsInRobot >= 3) {
                } else {
                    spindex.changeCurrentPositionBy(spindexThirdRevolution);
                }
                intakeState = INTAKE_STATE.EMPTY;

                break;

        }


    }
}


