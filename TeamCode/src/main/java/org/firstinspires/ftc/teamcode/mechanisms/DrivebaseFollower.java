package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.Constants.Constants.ranAuto;
import static org.firstinspires.ftc.teamcode.Constants.Constants.onBlueAlliance;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Line;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.function.Supplier;



public class DrivebaseFollower {

    private Follower follower;
    private Pose poseLock = new Pose();
    public static Pose startingPose = new Pose(88, 8, 90); //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private boolean automatedAim;
    private Supplier<PathChain> pathChain;
    private Supplier<PathChain> aimChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
//
    private Limelight3A limelight;
    private LLStatus llStatus;
    private LLResult llResults;

    private Telemetry telem;
//
    private double TX = 0;

    public DrivebaseFollower(HardwareMap hardwareMap) {
        telem = new MultipleTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); //TODO: Integrate softelectronics?
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(38.5, 33.5)))) //TODO: Update pose as needed
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 2))
                .build();


        TX = 0;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(onBlueAlliance ? 0 : 1);
    }

    public void drivebaseStart() {
        follower.startTeleopDrive();
    }

// Red Center park - 38.5, 33.5
 // Blue Center park - 105.5, 33.5

    public void driveLoop(double drive, double strafe, double turn) {
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -drive,
                    -strafe,
                    -turn,
                    false // Field Centric
            );
                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -drive * slowModeMultiplier,
                    -strafe * slowModeMultiplier,
                    -turn * slowModeMultiplier,
                    false // Field Centric
            );
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telem.addData("heading", follower.getHeading());
        telem.addData("slow?", slowMode);
        telem.addData("followerHeadingGoal", follower.getCurrentPathChain() != null ? follower.getCurrentPathChain().getPath(0).getHeadingGoal(3) : null);
    }

    public void parkPathfollowerLoop(boolean enable) {
        //Automated PathFollowing
        if (enable) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }
        //Stop automated following if the follower is done
        if (automatedDrive && (!enable || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        telemetryM.debug("automatedDrive", automatedDrive);
    }

    public void setPoseLock() {
        poseLock = new Pose(follower.getPose().getX(), follower.getPose().getY());
        aimChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierPoint(new Pose(poseLock.getX(), poseLock.getY()))))//Lazy Curve Generation
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(67), 3))
                .build();
    }

    public void autoAimLoop(boolean enable) {


        //Automated PathFollowing
        if (enable) {
            follower.followPath(aimChain.get());
            automatedAim = true;
        }
        //Stop automated following if the follower is done
        if (automatedAim && (!enable || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedAim = false;
        }

        telemetryM.debug("automatedDrive", automatedDrive);
    }

    public void toggleSlowMode() {
        slowMode = !slowMode;
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public Pose getPoseLock() {
        return poseLock;
    }

//    public void turnToGoal() {
//        double heading = Math.toDegrees(otos.getPosition().h);
//        if (onBlueAlliance) {
//            if (heading > 62 && heading < 72) {
//                drive (0, 0, 0);
//            } else {
//                drive (0, 0, -0.05 * (67-heading));
//            }
//        } else {
//            if (heading > 62 && heading < 72) {
//                drive (0, 0, 0);
//            } else {
//                drive (0, 0, -0.05 * (67-heading));
//            }
//        }

    public boolean getTXInRange() {return llResults.getFiducialResults().get(0).getTargetXDegrees() <= .5 && llResults.getFiducialResults().get(0).getTargetXDegrees() >= -.5;}
    public boolean getTargetSeen() {return llResults.isValid();}
}
