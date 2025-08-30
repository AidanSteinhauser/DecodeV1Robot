package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Auto extends OpMode {
    private Follower follower;
    private int pathState;
    private Timer pathTimer;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose poseOne = new Pose(0, 0, Math.toRadians(0));
    private final Pose poseTwo = new Pose(0, 0, Math.toRadians(0));
    private final Pose poseThree = new Pose(0, 0, Math.toRadians(0));
    private final Pose poseFour = new Pose(0, 0, Math.toRadians(0));

    private PathChain path1, path2, path3, path4;

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called once at the start of the OpMode. It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        setPathState(0);
    }

    private void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, poseOne))
                .setLinearHeadingInterpolation(startPose.getHeading(), poseOne.getHeading())
                .build();
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(poseOne, poseTwo))
                .setLinearHeadingInterpolation(poseOne.getHeading(), poseTwo.getHeading())
                .build();
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(poseTwo, poseThree))
                .setLinearHeadingInterpolation(poseTwo.getHeading(), poseThree.getHeading())
                .build();
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(poseThree, poseFour))
                .setLinearHeadingInterpolation(poseThree.getHeading(), poseFour.getHeading())
                .build();
    }




    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        // Loop robot movement and odometry values
        follower.update();
        // Loop the finite-state machine
        autonomousPathUpdate();
        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }




    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(path1, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(path2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(path3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(path4, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }
}
