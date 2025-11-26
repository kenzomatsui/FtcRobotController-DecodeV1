package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.objects.Intake;
import org.firstinspires.ftc.teamcode.drive.objects.ShooterObj;

@Autonomous(name = "Example Auto", group = "Examples")
public class PedroAutonomous extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private Intake intake;
    private ShooterObj shooter;

    private int pathState;

    // ---------------------- POSES ----------------------
    private final Pose startPose      = new Pose(24, 129, Math.toRadians(140));
    private final Pose scorePose      = new Pose(55, 85,  Math.toRadians(180));

    private final Pose pickup1Pose    = new Pose(60, 60,  Math.toRadians(180));
    private final Pose endPickup1Pose = new Pose(24, 85,  Math.toRadians(180));

    private final Pose pickup2Pose    = new Pose(60, 28,  Math.toRadians(180));
    private final Pose endPickup2Pose = new Pose(18, 28,  Math.toRadians(180));

    private final Pose pickup3Pose    = new Pose(60, -8,  Math.toRadians(180));
    private final Pose endPickup3Pose = new Pose(18, -8,  Math.toRadians(180));

    // ------------------- PATH OBJECTS -------------------
    private Path scorePreload;

    private Path slowGrabPickup1;
    private Path slowGrabPickup2;
    private Path slowGrabPickup3;

    private PathChain scorePickup1;
    private PathChain grabPickup2, scorePickup2;
    private PathChain grabPickup3, scorePickup3;

    // ------------------- BUILD PATHS -------------------
    public void buildPaths() {

        // ******************* PRELOAD → SCORE *******************
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());


        // ============================================================
        // ======================== PICKUP 1 ===========================
        // ============================================================
        slowGrabPickup1 = new Path(new BezierLine(scorePose, endPickup1Pose));
        slowGrabPickup1.setLinearHeadingInterpolation(scorePose.getHeading(), endPickup1Pose.getHeading());

        // velocidade em inches/second — ajuste entre 8 e 18
        slowGrabPickup1.setVelocityConstraint(12.0);
        slowGrabPickup1.setTimeoutConstraint(500);



        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(endPickup1Pose, scorePose))
                .setLinearHeadingInterpolation(endPickup1Pose.getHeading(), scorePose.getHeading())
                .build();


        // ============================================================
        // ======================== PICKUP 2 ===========================
        // ============================================================

        slowGrabPickup2 = new Path(new BezierLine(scorePose, pickup2Pose));
        slowGrabPickup2.setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading());
        slowGrabPickup2.setVelocityConstraint(8.0);
        slowGrabPickup2.setTimeoutConstraint(500);

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, endPickup2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), endPickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(endPickup2Pose, scorePose))
                .setLinearHeadingInterpolation(endPickup2Pose.getHeading(), scorePose.getHeading())
                .build();


        // ============================================================
        // ======================== PICKUP 3 ===========================
        // ============================================================

        slowGrabPickup3 = new Path(new BezierLine(scorePose, pickup3Pose));
        slowGrabPickup3.setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading());
        slowGrabPickup3.setVelocityConstraint(8.0);
        slowGrabPickup3.setTimeoutConstraint(500);

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, endPickup3Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), endPickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(endPickup3Pose, scorePose))
                .setLinearHeadingInterpolation(endPickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }


    // ------------------- STATE MACHINE -------------------
    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0: // PRELOAD
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            // ===================== PICKUP 1 =====================
            case 1:
                if (!follower.isBusy()) {
                    intake.Coleta(-1, 0);
                    follower.setMaxPower(0.3);
                    follower.followPath(slowGrabPickup1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    intake.Coleta(0, 0);
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;


            // ===================== PICKUP 2 =====================
            case 3:
                if (!follower.isBusy()) {
                    intake.Coleta(1, 1);
                    follower.followPath(slowGrabPickup2, true);
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    intake.Coleta(0, 0);
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;


            // ===================== PICKUP 3 =====================
            case 5:
                if (!follower.isBusy()) {
                    intake.Coleta(1, 1);
                    follower.followPath(slowGrabPickup3, true);
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    intake.Coleta(0, 0);
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;


            case 7:
                if (!follower.isBusy()) {
                    intake.Coleta(0, 0);
                    setPathState(-1);
                }
                break;
        }
    }


    // ------------------- UTILS -------------------
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    // ------------------- LOOP -------------------
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.update();
    }


    // ------------------- INIT -------------------
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        intake   = new Intake(hardwareMap);
        shooter  = new ShooterObj(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override public void init_loop() {}

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void stop() {}
}
