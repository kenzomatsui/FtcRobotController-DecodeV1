package org.firstinspires.ftc.teamcode.pedroPathing;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.objects.Intake;
import org.firstinspires.ftc.teamcode.drive.objects.ShooterObj;

@Autonomous(name = "Red Auto", group = "Examples")
public class RedAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;

    boolean indexer_work = false;
    boolean base_lock = false;

    private Intake intake;
    private ShooterObj shooter;

    private int pathState = 0;

    // ---------------------- POSES ----------------------
    private final Pose startPose       = new Pose(120, 129, Math.toRadians(40)); //120
    private final Pose scorePose       = new Pose(79, 54,  Math.toRadians(55));//86

    private final Pose pickup2Pose     = new Pose(97, 61,  Math.toRadians(0));//97
    private final Pose endPickup2Pose  = new Pose(126, 61,  Math.toRadians(0));//126

    private final Pose pickup3Pose     = new Pose(47, 37, Math.toRadians(0));
    private final Pose endPickup3Pose  = new Pose(18, 37, Math.toRadians(0));

    private final Pose endPickup1Pose  = new Pose(125, 85, Math.toRadians(0));//125

    // ------------------- PATHS -------------------
    private Path toScore;

    private Path toPickup1_high;
    private Path toPickup1_endSlow;
    private Path pickup1_toScore;

    private Path toPickup2_high;
    private Path toPickup2_endSlow;
    private Path pickup2_toScore;

    private Path toPickup3_high;
    private Path toPickup3_endSlow;
    private Path pickup3_toScore;

    // ------------------- BUILD ALL PATHS -------------------
    public void buildPaths() {

        // PRELOAD → SCORE
        toScore = new Path(new BezierLine(startPose, scorePose));
        toScore.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // PICKUP 1
        toPickup1_high = new Path(new BezierLine(scorePose, endPickup1Pose));
        toPickup1_high.setLinearHeadingInterpolation(scorePose.getHeading(), endPickup1Pose.getHeading());

        // (pickup1 não tem "pickupPose", então só há 1 path lento)
        toPickup1_endSlow = toPickup1_high;

        pickup1_toScore = new Path(new BezierLine(endPickup1Pose, scorePose));
        pickup1_toScore.setLinearHeadingInterpolation(endPickup1Pose.getHeading(), scorePose.getHeading());

        // PICKUP 2
        toPickup2_high = new Path(new BezierLine(scorePose, pickup2Pose));
        toPickup2_high.setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading());

        toPickup2_endSlow = new Path(new BezierLine(pickup2Pose, endPickup2Pose));
        toPickup2_endSlow.setLinearHeadingInterpolation(pickup2Pose.getHeading(), endPickup2Pose.getHeading());

        pickup2_toScore = new Path(new BezierLine(endPickup2Pose, scorePose));
        pickup2_toScore.setLinearHeadingInterpolation(endPickup2Pose.getHeading(), scorePose.getHeading());

        // PICKUP 3
        toPickup3_high = new Path(new BezierLine(scorePose, pickup3Pose));
        toPickup3_high.setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading());

        toPickup3_endSlow = new Path(new BezierLine(pickup3Pose, endPickup3Pose));
        toPickup3_endSlow.setLinearHeadingInterpolation(pickup3Pose.getHeading(), endPickup3Pose.getHeading());

        pickup3_toScore = new Path(new BezierLine(endPickup3Pose, scorePose));
        pickup3_toScore.setLinearHeadingInterpolation(endPickup3Pose.getHeading(), scorePose.getHeading());
    }

    // ------------------- STATE MACHINE -------------------
    public void autonomousPathUpdate() {

        switch (pathState) {

            // PRELOAD → SCORE
            case 0:
                follower.setMaxPower(1.0);
                follower.followPath(toScore);
                setPathState(1);
                break;

            // SCORE → PICKUP1 (alta velocidade + coleta lenta)
            case 1:
                if (!follower.isBusy()) {
                    sleep(1700);
                    shooter.SHOOTER3(true);

                    follower.setMaxPower(0.5);
                    follower.followPath(toPickup1_endSlow);
                    setPathState(2);

                    indexer_work = true;
                    intake.Coleta(-1,0);
                }
                break;

            // END PICKUP1 → SCORE
            case 2:
                if (!follower.isBusy()) {
                    intake.Coleta(0,0);

                    follower.setMaxPower(1);
                    follower.followPath(pickup1_toScore);
                    setPathState(3);
                }
                break;

            // SCORE → PICKUP2 HIGH SPEED
            case 3:
                if (!follower.isBusy()) {
                    shooter.SHOOTER3(true);

                    follower.setMaxPower(1);
                    follower.followPath(toPickup2_high);
                    setPathState(4);
                }
                break;

            // PICKUP2 SLOW
            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.3);
                    follower.followPath(toPickup2_endSlow);
                    setPathState(5);

                    indexer_work = true;
                    intake.Coleta(-1,0);
                }
                break;

            // END PICKUP2 → SCORE
            case 5:
                if (!follower.isBusy()) {
                    intake.Coleta(0,0);
                    follower.setMaxPower(1.0);
                    follower.followPath(pickup2_toScore);
                    setPathState(6);
                }
                break;

            // SCORE → PICKUP3 HIGH SPEED
            case 6:
                if (!follower.isBusy()) {
                    shooter.SHOOTER3(true);
                    follower.setMaxPower(1.0);
                    follower.followPath(toPickup3_high);
                    setPathState(7);
                }
                break;

            // PICKUP3 SLOW
            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.25);
                    follower.followPath(toPickup3_endSlow);
                    setPathState(8);
                }
                break;

            // END PICKUP3 → SCORE
            case 8:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(pickup3_toScore);
                    setPathState(9);
                }
                break;

            case 9:
                // FIM
                break;
        }
    }

    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    // ------------------- LOOP -------------------
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        shooter.setShooterPowerLow(0.75);

        if(indexer_work){
            shooter.testMotor();
        }

        telemetry.addData("State", pathState);
        telemetry.update();
    }

    // ------------------- INIT -------------------
    @Override
    public void init() {
        base_lock = true;
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        intake   = new Intake(hardwareMap);
        shooter  = new ShooterObj(hardwareMap);

        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override public void init_loop() {}
    @Override public void start() { setPathState(0); }
    @Override public void stop() {}
}
