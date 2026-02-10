package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.objects.Intake;
import org.firstinspires.ftc.teamcode.drive.objects.PedroPathingMotorController;
import org.firstinspires.ftc.teamcode.drive.objects.PedroPathingShooterController;
import org.firstinspires.ftc.teamcode.drive.objects.ShooterObjBlue;
import org.firstinspires.ftc.teamcode.pedroPathingVelho.Constants;

@Autonomous(name = "Blue Auto SK", group = "Autonomous")
public class BlueAutoSk extends OpMode {

    // ---------------- HW / CONTROLLERS ----------------
    private Follower follower;
    private Intake intakeObj;
    private ShooterObjBlue shooterObj;

    private PedroPathingMotorController turretController = new PedroPathingMotorController();
    private PedroPathingShooterController flywheelController = new PedroPathingShooterController();

    // Ajuste conforme seu hardware
    private static final String TURRET_MOTOR_NAME = "RMX";
    private static final String FLYWHEEL_MOTOR_NAME = "RMTa";

    // ---------------- TARGET (GOAL) ----------------
    //TODO: coloque o (X,Y) real do goal azul no seu sistema de coordenadas do campo (polegadas)
    private static final double goalX = 0;
    private static final double goalY = 114;

    // ---------------- STATE MACHINE ----------------
    private int state = 0;
    private final ElapsedTime stateTimer = new ElapsedTime();

    private void setState(int newState) {
        state = newState;
        stateTimer.reset();
    }

    // ---------------- POSES ----------------
    private final Pose startPose       = new Pose(24, 129, Math.toRadians(140));
    private final Pose scorePose       = new Pose(65, 90,  Math.toRadians(135));

    private final Pose endPickup1Pose  = new Pose(16, 85,  Math.toRadians(190));

    private final Pose pickup2Pose     = new Pose(47, 61,  Math.toRadians(180));
    private final Pose endPickup2Pose  = new Pose(18, 61,  Math.toRadians(180));

    private final Pose pickup3Pose     = new Pose(47, 37,  Math.toRadians(180));
    private final Pose endPickup3Pose  = new Pose(18, 37,  Math.toRadians(180));

    // ---------------- PATHS ----------------
    private Path toScore;

    private Path toPickup1;
    private Path pickup1_toScore;

    private Path toPickup2_high;
    private Path toPickup2_slow;
    private Path pickup2_toScore;

    private Path toPickup3_high;
    private Path toPickup3_slow;
    private Path pickup3_toScore;

    private void buildPaths() {
        toScore = new Path(new BezierLine(startPose, scorePose));
        toScore.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        toPickup1 = new Path(new BezierLine(scorePose, endPickup1Pose));
        toPickup1.setLinearHeadingInterpolation(scorePose.getHeading(), endPickup1Pose.getHeading());

        pickup1_toScore = new Path(new BezierLine(endPickup1Pose, scorePose));
        pickup1_toScore.setLinearHeadingInterpolation(endPickup1Pose.getHeading(), scorePose.getHeading());

        toPickup2_high = new Path(new BezierLine(scorePose, pickup2Pose));
        toPickup2_high.setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading());

        toPickup2_slow = new Path(new BezierLine(pickup2Pose, endPickup2Pose));
        toPickup2_slow.setLinearHeadingInterpolation(pickup2Pose.getHeading(), endPickup2Pose.getHeading());

        pickup2_toScore = new Path(new BezierLine(endPickup2Pose, scorePose));
        pickup2_toScore.setLinearHeadingInterpolation(endPickup2Pose.getHeading(), scorePose.getHeading());

        toPickup3_high = new Path(new BezierLine(scorePose, pickup3Pose));
        toPickup3_high.setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading());

        toPickup3_slow = new Path(new BezierLine(pickup3Pose, endPickup3Pose));
        toPickup3_slow.setLinearHeadingInterpolation(pickup3Pose.getHeading(), endPickup3Pose.getHeading());

        pickup3_toScore = new Path(new BezierLine(endPickup3Pose, scorePose));
        pickup3_toScore.setLinearHeadingInterpolation(endPickup3Pose.getHeading(), scorePose.getHeading());
    }

    // ---------------- HELPERS (não-bloqueantes) ----------------

    /** Liga intake por potência por um tempo; você pode ajustar valores conforme seu robô. */
    private void intakeOn(double power) {
        // sua Intake soma lt+rt e *2, então pra “fixo” usamos lt=power/2, rt=0 por ex.
        // (ou o contrário). Aqui: lt = power/2, rt = 0 => ps=(lt+rt)*2=power
        intakeObj.Coleta(power / 2.0, 0);
    }

    private void intakeOff() {
        intakeObj.Coleta(0, 0);
    }

    /** “Empurra” artefatos: usando o Shoot() atual (indexer com lógica do sensor). */
    private void feedIndexerFor(double seconds) {
        if (stateTimer.seconds() < seconds) {
            shooterObj.Shoot(1.0); // trigger > 0.3 => indexer = 1
        } else {
            shooterObj.Shoot(0.0);
        }
    }

    // ---------------- AUTONOMOUS STATE MACHINE ----------------
    private void autonomousUpdate() {
        switch (state) {

            // 0) START -> ir pro score
            case 0: {
                follower.setMaxPower(0.8);
                follower.followPath(toScore);
                setState(1);
                break;
            }

            // 1) chegou no score -> estabiliza e atira preload (não-bloqueante)
            case 1: {
                if (!follower.isBusy()) {
                    // opcional: espera curtinho o robô “assentar”
                    setState(2);
                }
                break;
            }

            // 2) SHOOT preload por ~1.2s (ajuste)
            case 2: {
                // durante o tiro, deixe intake ajudando a alimentar (se fizer sentido no seu hardware)
                intakeOn(-1.0); // cuidado com sentido; ajuste se necessário
                feedIndexerFor(1.2);

                if (stateTimer.seconds() >= 1.2) {
                    intakeOff();
                    shooterObj.Shoot(0.0);
                    setState(3);
                }
                break;
            }

            // 3) score -> pickup1
            case 3: {
                follower.setMaxPower(1.0);
                follower.followPath(toPickup1);
                setState(4);
                break;
            }

            // 4) enquanto vai/chega pickup1: liga intake; quando chegar, segura um pouco coletando
            case 4: {
                intakeOn(1.0);
                if (!follower.isBusy()) {
                    setState(5);
                }
                break;
            }

            // 5) “dwell” pickup1 (0.4s coletando parado)
            case 5: {
                intakeOn(1.0);
                if (stateTimer.seconds() >= 0.4) {
                    intakeOff();
                    follower.setMaxPower(1.0);
                    follower.followPath(pickup1_toScore);
                    setState(6);
                }
                break;
            }

            // 6) voltou pro score1 -> atira curto
            case 6: {
                if (!follower.isBusy()) setState(7);
                break;
            }

            case 7: {
                intakeOn(-1.0);
                feedIndexerFor(1.0);
                if (stateTimer.seconds() >= 1.0) {
                    intakeOff();
                    shooterObj.Shoot(0.0);
                    setState(8);
                }
                break;
            }

            // 8) score -> pickup2 (high)
            case 8: {
                follower.setMaxPower(1.0);
                follower.followPath(toPickup2_high);
                setState(9);
                break;
            }

            // 9) pickup2 slow
            case 9: {
                intakeOn(1.0);
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(toPickup2_slow);
                    setState(10);
                }
                break;
            }

            // 10) chegou pickup2 -> dwell
            case 10: {
                intakeOn(1.0);
                if (!follower.isBusy()) setState(11);
                break;
            }

            case 11: {
                intakeOn(1.0);
                if (stateTimer.seconds() >= 0.4) {
                    intakeOff();
                    follower.setMaxPower(1.0);
                    follower.followPath(pickup2_toScore);
                    setState(12);
                }
                break;
            }

            // 12) score2 -> shoot
            case 12: {
                if (!follower.isBusy()) setState(13);
                break;
            }

            case 13: {
                intakeOn(-1.0);
                feedIndexerFor(1.0);
                if (stateTimer.seconds() >= 1.0) {
                    intakeOff();
                    shooterObj.Shoot(0.0);
                    setState(14);
                }
                break;
            }

            // 14) score -> pickup3 (high)
            case 14: {
                follower.setMaxPower(1.0);
                follower.followPath(toPickup3_high);
                setState(15);
                break;
            }

            // 15) pickup3 slow
            case 15: {
                intakeOn(1.0);
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(toPickup3_slow);
                    setState(16);
                }
                break;
            }

            // 16) chegou pickup3 -> dwell
            case 16: {
                intakeOn(1.0);
                if (!follower.isBusy()) setState(17);
                break;
            }

            case 17: {
                intakeOn(1.0);
                if (stateTimer.seconds() >= 0.4) {
                    intakeOff();
                    follower.setMaxPower(1.0);
                    follower.followPath(pickup3_toScore);
                    setState(18);
                }
                break;
            }

            // 18) score3 -> shoot
            case 18: {
                if (!follower.isBusy()) setState(19);
                break;
            }

            case 19: {
                intakeOn(-1.0);
                feedIndexerFor(1.0);
                if (stateTimer.seconds() >= 1.0) {
                    intakeOff();
                    shooterObj.Shoot(0.0);
                    setState(20);
                }
                break;
            }

            // 20) fim
            case 20: {
                intakeOff();
                shooterObj.Shoot(0.0);
                // flywheelController.stop(); // Desligar o flywheel no final
                break;
            }
        }
    }

    // ---------------- FTC OPMODE ----------------
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        intakeObj = new Intake(hardwareMap);
        shooterObj = new ShooterObjBlue(hardwareMap);

        buildPaths();

        // Turret + flywheel mirando no goal via odometria
        turretController.init(hardwareMap, follower, TURRET_MOTOR_NAME);
        turretController.setTargetPosition(goalX, goalY);
        turretController.setLimits(-60.0, 260.0);

        flywheelController.init(hardwareMap, follower, FLYWHEEL_MOTOR_NAME);
        flywheelController.setTargetPosition(goalX, goalY);
        flywheelController.setPowerConfig(0.25, 0.90, 28.0, 120.0);

        telemetry.addData("Status", "Init OK");
        telemetry.update();
    }

    @Override
    public void start() {
        setState(0);
    }

    @Override
    public void loop() {
        follower.update();

        // manter mirando e ajustando flywheel sempre (odometria)
        turretController.update();
        flywheelController.update();

        autonomousUpdate();

        telemetry.addData("State", state);
        telemetry.addData("PoseX", follower.getPose().getX());
        telemetry.addData("PoseY", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("FlywheelPower", flywheelController.getCurrentPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeOff();
        shooterObj.Shoot(0.0);
        flywheelController.stop();
    }
}
