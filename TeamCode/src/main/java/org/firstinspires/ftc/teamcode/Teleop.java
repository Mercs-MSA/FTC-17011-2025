package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.Constants.CLOSE_SHOT_VELOCITY;
import static org.firstinspires.ftc.teamcode.Constants.Constants.FAR_SHOT_VELOCITY;
import static org.firstinspires.ftc.teamcode.Constants.Constants.blueGoal;
import static org.firstinspires.ftc.teamcode.Constants.Constants.currentTheta;
import static org.firstinspires.ftc.teamcode.Constants.Constants.currentX;
import static org.firstinspires.ftc.teamcode.Constants.Constants.currentY;
import static org.firstinspires.ftc.teamcode.Constants.Constants.onBlueAlliance;
import static org.firstinspires.ftc.teamcode.Constants.Constants.ranAuto;
import static org.firstinspires.ftc.teamcode.Constants.Constants.redGoal;
import static org.firstinspires.ftc.teamcode.mechanisms.Shooter.turVelPIDF;
import static org.firstinspires.ftc.teamcode.mechanisms.Shooter.turretP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Drivebase;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Config
@TeleOp(name = "Teleop", group = "Competition")
public class Teleop extends OpMode {

    // Dashboard / telemetry
    private FtcDashboard dash;
    private SoftElectronics softElectronics;
    private static Telemetry myTelem;
    private static TelemetryManager myPanels;

    // Mechanisms
    private Drivebase drivebase;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;

    // Drive input
    private double drive = 0;
    private double strafe = 0;
    private double turn = 0;


    // Shooter velocities
//    public static int FAR_SHOT_VELOCITY = 1967;
//    public static int CLOSE_SHOT_VELOCITY = 1619;
    public static int UPPER = 2200;
    public static int LOWER = 0;

    public static int shooterDesiredVelocity = 0;

    public static double redTargetAngleFar = 67;
    public static double redTargetAngleClose = 80;
    public static double blueTargetAngleFar = -67;
    public static double blueTargetAngleClose = -80;


    public enum STARTING_ORIENTATION {
        GOAL_SIDE,
        PLAYER_SIDE
    }

    public enum SHOOTING_STATE {
        INACTIVE,
        START,
        SPIN_UP,
        SHOOT,
        END
    }
    public static SHOOTING_STATE shootingState = SHOOTING_STATE.INACTIVE;
    public static STARTING_ORIENTATION startingOrientation = STARTING_ORIENTATION.GOAL_SIDE;

    private enum TURRET_STATE {
        GO_TO_ZERO,
        ZEROED,
        AIMED,
        AIMING_NO_TAG,
        AIMING_TO_TAG
    }

    TURRET_STATE turretState = TURRET_STATE.ZEROED;

    @Override
    public void init() {
        softElectronics = new SoftElectronics(hardwareMap, this.telemetry);
        dash = FtcDashboard.getInstance();
        myTelem = new MultipleTelemetry(dash.getTelemetry(), softElectronics.getTelemetry());
        myPanels = softElectronics.getPanelsTelemetry();

        drivebase = new Drivebase(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        shootingState = SHOOTING_STATE.INACTIVE;
        turretState = TURRET_STATE.ZEROED;

        shooterDesiredVelocity = 0;

        myTelem.addData("Status", "Initialized");
        myTelem.update();

        if (!ranAuto) {
            if (!onBlueAlliance)
                drivebase.setPosition(new SparkFunOTOS.Pose2D(88, 8, Math.toRadians(90)));
            else
                drivebase.setPosition(new SparkFunOTOS.Pose2D(56, 8, Math.toRadians(90)));
        } else {
            drivebase.setPosition(new SparkFunOTOS.Pose2D(currentX, currentY, currentTheta));
        }
    }

    @Override
    public void init_loop() {
        super.init_loop();
        if (gamepad1.rightBumperWasPressed()) {
            onBlueAlliance = true;
        } else if (gamepad1.leftBumperWasPressed()) {
            onBlueAlliance = false;
        }

        if (onBlueAlliance) {
            telemetry.addLine("Blue alliance selected. Press gamepad 1 left bumper to switch.");
            drivebase.offsetYaw(-90);
        } else {
            telemetry.addLine("Red alliance selected. Press gamepad 1 right bumper to switch.");
            drivebase.offsetYaw(90);
        }
    }

    @Override
    public void start() {
//        shooter.setMotorVelocity(300);
        turretState = TURRET_STATE.AIMING_NO_TAG;

        shooter.setMotorVelocity(0);
        ranAuto = false;
    }

    @Override
    public void loop() {
        updateDrivebase();
        updateMechanisms();
        updateTurretState();
        updateTelemetry();

//        drivebase.updateLL();
    }

    private void updateDrivebase() {
        if (gamepad1.left_stick_button || gamepad1.right_stick_button) {
            drive = -gamepad1.left_stick_y * .5;
            strafe = gamepad1.left_stick_x * .5;
            turn = gamepad1.right_stick_x * .5;
        } else {
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
        }

        drivebase.drive(drive, strafe, turn);

        if (gamepad1.dpad_up) {
            drivebase.resetYaw();
        }

        if (gamepad1.triangle) {
//            drivebase.turnToGoal();
        }
    }


    private void updateMechanisms() {
        shootingMachine();
        setShooterDesiredVelocity(); ///STILL REQUIRES TUNING

//        if (gamepad1.dpad_down) {
//            shooterDesiredVelocity = FAR_SHOT_VELOCITY;
//        } else {
//            shooterDesiredVelocity = CLOSE_SHOT_VELOCITY;
//        }

        /// FOR TESTING
//        if (gamepad1.cross) {
//            shooter.setMotorVelocity(shooterDesiredVelocity);
//        } else {
//            shooter.setMotorVelocity(0);
//        }

        if (gamepad1.right_trigger > 0.3 && shootingState.equals(SHOOTING_STATE.INACTIVE)) {
            shootingState = SHOOTING_STATE.START;
        }
//            transfer.openTransferGate();
//            intake.setPower(1);
//            transfer.setPower(1);
//        } else if (!gamepad1.left_bumper && !gamepad1.right_bumper) { /// FOR TESTING
//            intake.setPower(0);
//            transfer.setPower(0);
//        }

//        if (gamepad1.left_trigger > 0.3) {
//            turretState = TURRET_STATE.AIMING_TO_TAG;
//        } else {
//            turretState = TURRET_STATE.ZEROED;
//        }

        if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            intake.setPower(1);
            transfer.setPower(1);
//            transfer.closeTransferGate();
        } else if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            intake.setPower(-1);
            transfer.setPower(-1);
        } else if (!shootingState.equals(SHOOTING_STATE.INACTIVE)) {
        } else {
            intake.setPower(0);
            transfer.setPower(0);
        }

        if (gamepad1.crossWasPressed()) {
            shooter.setTurretPIDF();
        }
    }

    private void updateTelemetry() {
        double shooterVel = shooter.getVelocity();
        double turretFieldHeading = Math.toDegrees(drivebase.getLaserHeading()) + (double) shooter.getTurretPos() / 8.13333333333; //TODO: Figure out why this value is constantly getting closer to 0
        myTelem.addData("Laser Range, inches: ", drivebase.distanceToTarget());
//        myTelem.addData("Lime Range, inches: ", drivebase.limeDistance());
        myTelem.addData("X, Y: ", drivebase.getPosition().x + ", " + drivebase.getPosition().y);
        myTelem.addData("Heading:", Math.toDegrees(drivebase.getPosition().h));
        myTelem.addData("Intake Power:", intake.getPower());
        myTelem.addData("Transfer Power:", transfer.getPower());
        myTelem.addData("Gate position: ", transfer.getTransferPosition());
        myTelem.addData("Shooting state: ", shootingState);
        myTelem.addData("Turret P: ", turretP);
        myTelem.addData("Turret PIDF: ", turVelPIDF);
        myTelem.addData("Turret state:", turretState);
        myTelem.addData("Reached desired velocity? ", desiredVelocityReached());
//        myTelem.addData("Tx:", drivebase.getLLResult().getTx());
        myTelem.addData("Turret velocity:", shooter.getTurretVelocity());
//        myTelem.addData("Turret heading PID", shooter.getTurretPositionalPID());
        myTelem.addData("Shooter current velocity: ", shooterVel);
        myTelem.addData("Shooter Target Vel:", shooterDesiredVelocity);
        myTelem.addData("Upper: ", UPPER);
        myTelem.addData("Lower: ", LOWER);
        myTelem.addData("Shooter Current (AMPS): ", shooter.getShooterCurrent());
        myTelem.update();
    }

    private void setShooterDesiredVelocity() {
        double range = drivebase.distanceToTarget();
        int velocity = 0;
        if (range < 80)
            velocity = (int) ((0.0586009 * Math.pow(range, 2)) + (-4.2766 * range) + 1498.02814);
        else
            velocity = (int) ((0.0227675 * Math.pow(range, 2)) + (1.62765 * range) + 1344.59538);

        shooterDesiredVelocity = Math.min(velocity, 2000);
    }

    private void OLDshooterDesiredVelocity() {
        double range = drivebase.distanceToTarget();
        shooterDesiredVelocity = (int) ((-0.0467391 * Math.pow(range, 2)) + (15.8207 * range) + 747.86042);
    }
    private boolean desiredVelocityReached() {
        return (shooter.getVelocity() > shooterDesiredVelocity * .97);
    }

    private void shootingMachine() {
        switch (shootingState) {
            case START:
                shooter.setMotorVelocity(shooterDesiredVelocity);
                shootingState = SHOOTING_STATE.SPIN_UP;
                break;
            case SPIN_UP:
                transfer.closeTransferGate();
                shooter.setMotorVelocity(shooterDesiredVelocity);
                if (desiredVelocityReached() && gamepad1.right_trigger > .3) {
                    shootingState = SHOOTING_STATE.SHOOT;
                } else if (!desiredVelocityReached() && gamepad1.right_trigger > .3) { /// THIS IS IMPORTANT!!!!!!!! IT KEEPS THE SHOOTER VELOCITY RAMPING
                } else {
                    shootingState = SHOOTING_STATE.END;
                }
                break;
            case SHOOT:
                transfer.openTransferGate();
                transfer.setPower(.87);
                shooter.setMotorVelocity(shooterDesiredVelocity);
                if (gamepad1.right_trigger > .3 && !desiredVelocityReached()) {
                    shootingState = SHOOTING_STATE.SPIN_UP;
                } else if (gamepad1.right_trigger <= .3) {
                    shootingState = SHOOTING_STATE.END;
                }
                break;
            case END:
//                shooter.setMotorVelocity(300);
                shooter.setMotorVelocity(0);
                transfer.closeTransferGate();
                transfer.setPower(0);
                shootingState = SHOOTING_STATE.INACTIVE;
                break;
            case INACTIVE:
                transfer.closeTransferGate();
                break;
        }
    }


    public void updateTurretState() {//Turret 180: -1484 //Turrent 360: -2954
        // Red goal: -42.6
//        double headingDeg = Math.toDegrees(drivebase.getLaserHeading());

        double targetFieldAngle = 0;
        if (onBlueAlliance) {
            targetFieldAngle = Math.toDegrees(Math.atan2(blueGoal.y - drivebase.getPosition().y, blueGoal.x - drivebase.getPosition().x));
        } else {
            targetFieldAngle = Math.toDegrees(Math.atan2(redGoal.y - drivebase.getPosition().y, redGoal.x - drivebase.getPosition().x));
        }
//        myTelem.addLine("Red (Y, X): " + (redGoal.y - drivebase.getPosition().y) + ", " + (redGoal.x - drivebase.getPosition().x) + " atan: " + Math.atan2(redGoal.y - drivebase.getPosition().y, redGoal.x - drivebase.getPosition().x));
//        myTelem.addData("Target Field Angle: ", targetFieldAngle);

        double robotHeadingDeg = Math.toDegrees(drivebase.getLaserHeading());
//        myTelem.addData("Robot Heading Degrees: ", robotHeadingDeg);

        double turretSetpointDeg = targetFieldAngle - robotHeadingDeg;
//        myTelem.addData("Turret Setpoint Degrees: ", turretSetpointDeg);

        double turretAngleDeg = shooter.getTurretPos() / 8.13333333333;
        myTelem.addData("Turret Angle Degrees: ", turretAngleDeg);

        double error = AngleUnit.normalizeDegrees(turretSetpointDeg - turretAngleDeg);
//        myTelem.addData("Turret Error: ", error);

        double absError = Math.abs(error);

//        boolean seesTarget = drivebase.getTargetSeen();
        boolean withinAngleLimit = Math.abs(turretAngleDeg) < 112 && Math.abs(targetFieldAngle) < 112;

//        double turretFieldHeading = AngleUnit.normalizeDegrees(headingDeg + turretDeg);
        switch (turretState) {
            case GO_TO_ZERO:
                shooter.setTurretVelocity(200, 1);
                shooter.setTurretTarget(0);
                if (shooter.getTurretPos() > -10 && shooter.getTurretPos() < 10)
                    turretState = TURRET_STATE.ZEROED;

            case ZEROED:
                shooter.setTurretVelocity(0, 0);

//                if (seesTarget && Math.abs(drivebase.getLLResult().getTx()) > 2)
//                    turretState = TURRET_STATE.AIMING_TO_TAG;
//                turretFieldHeading = Math.toDegrees(drivebase.getLaserHeading());
                if (gamepad1.left_trigger < .3)
                    turretState = TURRET_STATE.AIMING_NO_TAG;
                break;

            case AIMED:
                shooter.setTurretVelocity(0, 0);

                if (gamepad1.left_trigger > .3) {
                    turretState = TURRET_STATE.GO_TO_ZERO;
//                } else if (seesTarget && Math.abs(drivebase.getLLResult().getTx()) > 1) {
//                    turretState = TURRET_STATE.AIMING_TO_TAG;
                } else if (withinAngleLimit && Math.abs(error) > 1) {
                    turretState = TURRET_STATE.AIMING_NO_TAG;
                } 

//                if (absError > 3 && seesTarget) {
//                    turretState = TURRET_STATE.AIMING_TO_TAG;
//                } else if (absError > 3) {
//                    turretState = TURRET_STATE.AIMING_NO_TAG;
//                }
                break;

            case AIMING_NO_TAG:
//                if (seesTarget) {
//                    turretState = TURRET_STATE.AIMING_TO_TAG;
//                    return;
//                }

                if (Math.abs(error) > .5) {
//                    if (!withinAngleLimit)
//                        shooter.setTurretVelocity(0, 0);
//                    else
//                        shooter.setTurretVelocity((int)(error * 100 + 25), 1);
                        shooter.setTurretTargetShortestPath(turretSetpointDeg);
                } else {
                    turretState = TURRET_STATE.AIMED;
                }

//                if (absError > 3) {
//                    shooter.setTurretTargetShortestPath(turretSetpointDeg);
//                } else if (absError < 2) {
//                    turretState = TURRET_STATE.AIMED;
//                }
//                myTelem.addData("turret target", turretSetpointDeg);
//                myTelem.addData("turret angle", turretAngleDeg);
//                myTelem.addData("error")
                break;

            case AIMING_TO_TAG:
                shooter.setTurretMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                shooter.setTurretVelocity(-(int)((drivebase.getLLResult().getTx()+2)*50), 0.2);
                double limeError = 0;
//                if (seesTarget)
//                    limeError = AngleUnit.normalizeDegrees(drivebase.getLLResult().getTx());
//                else {
//                    turretState = TURRET_STATE.AIMING_NO_TAG;
//                    return;
//                }

                double absLimeError = Math.abs(limeError);

                if (absLimeError > 1.75) {
//                    shooter.setTurretTargetShortestPath(limeError);
                    if (!withinAngleLimit)
                        shooter.setTurretVelocity(0,0);
                    else
                        shooter.setTurretVelocity((int)(-limeError * 100 + 25), 1);
                } else {
                    turretState = TURRET_STATE.AIMED;
                }
                break;

            default:
                break;
        }
//        myTelem.addData("fieldTurretHeadihng", turretFieldHeading);
    }

}