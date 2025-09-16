package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Simple Path Test", group = "Test")
public class TestOpmode extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Pose definitions
    private final Pose startPose = new Pose(0, 0, Math.toRadians(90));
    private final Pose pose1 = new Pose(12, 12, Math.toRadians(45));
    private final Pose pose2 = new Pose(12, 60, Math.toRadians(90));
    private final Pose endPose = new Pose(0, 0, Math.toRadians(180));

    // PathChain definitions
    private PathChain path1;  // Start to (12,12)
    private PathChain path2;  // (12,12) to (24,60)
    private PathChain path3;  // (24,60) back to (0,0)

    public void preStart() {
        // Build all paths using the new format
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startPose,
                                pose1
                        )
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), pose1.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                pose1,
                                pose2
                        )
                )
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                pose2,
                                endPose
                        )
                )
                .setLinearHeadingInterpolation(pose2.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Move from (0,0) to (12,12)
                if (!follower.isBusy()) {
                    follower.followPath(path1, true);
                    setPathState(1);
                }
                break;
            case 1:
                // Move from (12,12) to (24,60)
                if (!follower.isBusy()) {
                    follower.followPath(path2, true);
                    setPathState(2);
                }
                break;
            case 2:
                // Move from (24,60) back to (0,0)
                if (!follower.isBusy()) {
                    follower.followPath(path3, true);
                    setPathState(3);
                }
                break;
            case 3:
                // Finished - stop execution
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        preStart();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        publishTelemetry();
    }

    public void publishTelemetry() {
        telemetry.addData("Path State", pathState);
        telemetry.addData("Current X", "%.2f", follower.getPose().getX());
        telemetry.addData("Current Y", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading (degrees)", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Is Busy", follower.isBusy());

        // Show target positions for debugging
        switch(pathState) {
            case 0:
                telemetry.addData("Target", "Going to (12, 12)");
                break;
            case 1:
                telemetry.addData("Target", "Going to (24, 60)");
                break;
            case 2:
                telemetry.addData("Target", "Going to (0, 0)");
                break;
            default:
                telemetry.addData("Target", "Finished");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        // Clean up if needed
    }
}