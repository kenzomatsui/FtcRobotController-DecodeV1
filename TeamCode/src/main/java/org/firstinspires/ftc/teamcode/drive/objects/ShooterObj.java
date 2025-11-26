package org.firstinspires.ftc.teamcode.drive.objects;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.actuators.MotorRPMWatcher;

public class ShooterObj {

    private static final double DISTANCIA_BOLA = 138; // mm

    private static final double KP = 0.08, KI = 0.0, KD = 0.002;
    private static final double TOLERANCIA = 0.5, XMAX_POWER = 0.5;

    private static final double MIN_POWER = 0.3, MAX_POWER = 1.0;
    private static final double TARGET_TA = 5.0, SCALE_FACTOR = 0.084;

    private Limelight3A limelight;
    public DistanceSensor sensorDistance;

    private DcMotorEx indexer, rotationMotorX, shooterD, intake;

    private double integral = 0, lastError = 0;
    private long lastTime = System.currentTimeMillis();

    boolean ballLoaded = false;
    long detectStart = 0;
    boolean timing = false;

    // ---------------- CONSTRUTOR ----------------

    public ShooterObj(HardwareMap hardwareMap) {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(7);

        rotationMotorX = hardwareMap.get(DcMotorEx.class, "RMX");
        rotationMotorX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterD = hardwareMap.get(DcMotorEx.class, "RMTa");
        shooterD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterD.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        indexer = hardwareMap.get(DcMotorEx.class, "index");

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
    }

    // ---------------- UPDATE ----------------

    public void update() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            alignToTarget(result);
            controlShooterPower(result);
        } else {
            stopAll();
        }
    }

    // ---------------- ALINHAMENTO ----------------

    private void alignToTarget(LLResult result) {

        double tx = result.getTx();
        double error = tx;

        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0;

        integral += error * dt;
        double derivative = (error - lastError) / dt;

        double power = KP * error + KI * integral + KD * derivative;
        power = Math.max(-XMAX_POWER, Math.min(power, XMAX_POWER));

        if (Math.abs(error) > TOLERANCIA) {
            rotationMotorX.setPower(power);
        } else {
            rotationMotorX.setPower(0);
            integral = 0;
        }

        lastError = error;
        lastTime = now;
    }

    // ---------------- CONTROLE DO SHOOTER ----------------

    private void controlShooterPower(LLResult result) {
        double ta = result.getTa();

        double distanceError = TARGET_TA - ta;
        double power = MIN_POWER + (distanceError * SCALE_FACTOR);

        power = Math.max(MIN_POWER, Math.min(power, MAX_POWER));

        if (ta >= TARGET_TA) {
            shooterD.setPower(0);
        } else {
            shooterD.setPower(power);
        }
    }


    // ---------------- DETECÇÃO DA BOLA ----------------

    public void detectBall() {

        if (ballLoaded) {
            indexer.setPower(0);
            return;
        }

        double distance = sensorDistance.getDistance(DistanceUnit.MM);

        if (distance < DISTANCIA_BOLA) {

            if (!timing) {
                detectStart = System.currentTimeMillis();
                timing = true;
                indexer.setPower(0.7);
            }

            if (System.currentTimeMillis() - detectStart >= 650) {
                indexer.setPower(0);
                ballLoaded = true;
                timing = false;
            }

        } else {
            indexer.setPower(0.7);
            timing = false;
        }
    }


    // ---------------- SHOOT ----------------

    public void Shoot(double Lp) {

        if (Lp > 0.01) {

            indexer.setPower(1);
            ballLoaded = false;
            return;
        }

        if (!ballLoaded) {
            indexer.setPower(0.7);
        } else {
            indexer.setPower(0);
        }
    }
    int shootState = 0;
    long shootTimer = 0;
    int shotsFired = 0;

    public void Shoota3(boolean shootButton) {

        switch (shootState) {

            // Espera o comando para começar os 3 tiros
            case 0:
                if (shootButton) {
                    shotsFired = 0;
                    shootState = 1;
                }
                break;

            // Garante que a bola está carregada antes de atirar
            case 1:
                detectBall();   // usa seu método normal
                if (ballLoaded) {
                    shootState = 2;
                } else {
                    intake.setPower(-0.7); // agora está CORRETO → puxa a bola pra dentro
                }
                break;

            // Atira a bola
            case 2:
                Shoot(1);
                intake.setPower(-1);   // CORRIGIDO → empurra a bola na direção certa

                ballLoaded = false;
                shootTimer = System.currentTimeMillis();
                shootState = 3;
                break;

            // Intervalo entre tiros → agora 1000 ms (1 segundo)
            case 3:
                if (System.currentTimeMillis() - shootTimer >= 1000) {

                    shotsFired++;

                    if (shotsFired < 3) {
                        shootState = 1;  // preparar próxima bola
                    } else {
                        shootState = 4;  // terminou
                    }
                }
                break;

            // Fim da sequência
            case 4:
                intake.setPower(0);
                indexer.setPower(0);
                shootState = 0;
                break;
        }
    }





    // ---------------- AUXILIARES ----------------

    public double getDistance() {
        return sensorDistance.getDistance(DistanceUnit.MM);
    }

    public void stopAll() {
        rotationMotorX.setPower(0);
        shooterD.setPower(0);
    }

    public void aimAndShoot() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            alignToTarget(result);
            controlShooterPower(result);
        } else {
            stopAll();
        }
    }
}
