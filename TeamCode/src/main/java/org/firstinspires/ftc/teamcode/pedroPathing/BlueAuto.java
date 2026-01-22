package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.objects.Intake;
import org.firstinspires.ftc.teamcode.drive.objects.PedroPathingMotorController;
import org.firstinspires.ftc.teamcode.drive.objects.PedroPathingShooterController;
import org.firstinspires.ftc.teamcode.drive.objects.ShooterObjBlue;

@Autonomous(name = "Blue Auto", group = "Examples")
public class BlueAuto extends OpMode {

    private Follower follower;
    private Follower followerpp;
    private Timer pathTimer;

    boolean indexer_work = false;
    boolean base_lock = false;

    private Intake intake;
    private ShooterObjBlue shooter;

    private PedroPathingMotorController turretController = new PedroPathingMotorController();
    private PedroPathingShooterController shooterController = new PedroPathingShooterController();
    private static final String MOTOR_NAME = "RMX";
    private static final String SHOOTER_MOTOR = "RMTa"; // Ajuste conforme seu hardware


    // Alvo inicial: gol azul
    private double targetX = 0;
    private double targetY = 125;


    private int pathState = 0;

    // ---------------------- POSES ----------------------
    private final Pose startPose       = new Pose(21, 124, Math.toRadians(140)); //120
    private final Pose scorePose       = new Pose(59, 83,  Math.toRadians(180));//86

    private final Pose pickup2Pose     = new Pose(47, 61,  Math.toRadians(180));//97
    private final Pose endPickup2Pose  = new Pose(18, 61,  Math.toRadians(180));//126

    private final Pose pickup3Pose     = new Pose(47, 37, Math.toRadians(180));
    private final Pose endPickup3Pose  = new Pose(18, 37, Math.toRadians(180));

    private final Pose endPickup1Pose  = new Pose(16, 85, Math.toRadians(185));//125

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
                setPathState(10);
                break;

            case 10:
                if (!follower.isBusy()) {
                    shooter.SHOOTER3(true);
                    setPathState(1);
                }
            // SCORE → PICKUP1 (baixa velocidade + coleta lenta)
            case 1:
                if (!follower.isBusy()) {
                    intake.intake.setPower(-1);
                    follower.setMaxPower(0.3);
                    follower.followPath(toPickup1_endSlow);
                    setPathState(2);
                }
                break;

            // END PICKUP1 → SCORE
            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(pickup1_toScore);
                    setPathState(3);
                }
                break;
            case 11:
                if (!follower.isBusy()){
                    shooter.SHOOTER3(true);
                    setPathState(3);
                }

            // SCORE → PICKUP2 HIGH SPEED
            case 3:
                if (!follower.isBusy()) {
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
                }
                break;

            // END PICKUP2 → SCORE
            case 5:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(pickup2_toScore);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()){
                    shooter.SHOOTER3(true);
                }

            // SCORE → PICKUP3 HIGH SPEED
            case 6:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toPickup3_high);
                    setPathState(7);
                }
                break;

            // PICKUP3 SLOW
            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
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
        followerpp.update();
        autonomousPathUpdate();

        turretController.update();
        shooterController.update();

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
        shooter  = new ShooterObjBlue(hardwareMap);

        follower.setStartingPose(startPose);
        buildPaths();

        // 1. Inicializar Pedro Pathing Follower
        // Certifique-se de que sua classe Constants está configurada corretamente
        followerpp = Constants.createFollower(hardwareMap);
        followerpp.setStartingPose(new Pose(24, 129, 140)); // Ajuste conforme sua posição inicial

        // 2. Inicializar Controlador da Turret
        turretController.init(hardwareMap, followerpp, MOTOR_NAME);
        turretController.setTargetPosition(targetX, targetY);
        turretController.setLimits(-60.0, 260.0);

        // 3. Inicializar Shooter
        shooterController.init(hardwareMap, followerpp, SHOOTER_MOTOR);
        shooterController.setTargetPosition(targetX, targetY);
        // Configura: MinPower 0.35 a 24pol, MaxPower 0.9 a 120pol
        shooterController.setPowerConfig(0.25, 0.90, 28.0, 120.0);
    }

    @Override public void init_loop() {}
    @Override public void start() { setPathState(0); }
    @Override public void stop() {}
}
