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

public class ShooterObj {

    private static final double DISTANCIA_BOLA = 138; // mm sensor de ball no slot final

    private static final double KP = 0.006, KI = 0.0, KD = 0.0025;
    private static final double TOLERANCIA = 0.5, XMAX_POWER = 0.5;

    private static final double MIN_POWER = 0.4, MAX_POWER = 1.0;
    private static final double TARGET_TA = 5.0, SCALE_FACTOR = 0.08;

    // Tempos (ajustáveis)
    private static final long SHOOTER_RECOVERY_TIME = 900;     // tempo para o shooter recuperar
    private static final long INDEXER_SHOOT_TIME    = 700;     // tempo que o indexer gira "de uma vez" para atirar
    private static final long INDEXER_ADVANCE_TIME  = 350;     // tempo para avançar durante carregamento
    private static final long INDEXER_DETECT_TIME   = 500;     // tempo de confirmação do sensor

    private Limelight3A limelight;
    public DistanceSensor sensorDistance;

    private DcMotorEx indexer, rotationMotorX, shooterD, intake;

    private double integral = 0, lastError = 0;
    private long lastTime = System.currentTimeMillis();

    // Controle interno
    public boolean temBola = false;
    private boolean timing = false;
    private long detectStart = 0;

    public ShooterObj(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(7);

        rotationMotorX = hardwareMap.get(DcMotorEx.class, "RMX");
        rotationMotorX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotorX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterD = hardwareMap.get(DcMotorEx.class, "RMTa");
        shooterD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterD.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Force a consistent direction for intake; use positive power to pull
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        indexer = hardwareMap.get(DcMotorEx.class, "index");
        // Make indexer run without encoder to ensure consistent timed spins
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
    }


    // ------------------- LIMELIGHT UPDATE ----------------------

    public void update() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            alignToTarget(result);
            controlShooterPower(result);
        } else stopAll();
    }


    // ------------------- ALINHAMENTO ---------------------------

    public void alignToTarget(LLResult result) {

        double tx = result.getTx();
        double error = tx;

        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0;

        integral += error * dt;
        double derivative = (error - lastError) / dt;

        double power = (KP * error + KI * integral + KD * derivative);
        power = Math.max(-XMAX_POWER, Math.min(power, XMAX_POWER));

        if (Math.abs(error) > TOLERANCIA) {
            rotationMotorX.setPower(power);
        } else {
            rotationMotorX.setPower(0);
            integral = 0;
        }
        rotationMotorX.setPower(power);

        lastError = error;
        lastTime = now;
    }

    //public double get_power() {return power;}
    public void SHOOTER3(boolean vai){
        intake.setPower(-1);
        if (vai) {
            sleep(500);
            Shoot(1);
            sleep(200);
            Shoot(1);
            sleep(200);
            Shoot(1);
            sleep(200);
            Shoot(1);
        }
    }


    // ------------------- CONTROLE DO SHOOTER -------------------

    private void controlShooterPower(LLResult result) {
        double ta = result.getTa();
        double distanceError = TARGET_TA - ta;
        double power = MIN_POWER + (distanceError * SCALE_FACTOR);

        power = Math.max(MIN_POWER, Math.min(power, MAX_POWER));

        if (ta >= TARGET_TA) {
            shooterD.setPower(0);
        }else{
            shooterD.setPower(power);
        }
    }


    // ------------------- DETECÇÃO DE BOLA ----------------------
    public void testMotor(){
        double dist = sensorDistance.getDistance(DistanceUnit.MM);

        if (dist < DISTANCIA_BOLA){
            sleep(500);
            indexer.setPower(0);
        }else{
            indexer.setPower(0.9);
        }
    }
    public void detectBall() {

        double dist = sensorDistance.getDistance(DistanceUnit.MM);

        if (dist < DISTANCIA_BOLA) {

            if (!timing) {
                detectStart = System.currentTimeMillis();
                timing = true;
                // Enquanto detectando, deixamos indexer empurrando (para estabilizar)
                indexer.setPower(0.7);
            }

            if (System.currentTimeMillis() - detectStart >= INDEXER_DETECT_TIME) {
                indexer.setPower(0);
                temBola = true;   // bola posicionada para tiro
                timing = false;
            }

        } else {
            // limpa temporizadores se não detecta
            timing = false;
        }
    }

    public double getDistance() {
        return sensorDistance.getDistance(DistanceUnit.MM);
    }


    // ------------------- TIRO MANUAL ---------------------------

    public void Shoot(double trigger) {

        if (trigger > 0.05) {
            // Giro "firme" e direto para empurrar a bola até o shooter
            indexer.setPower(1.0);
            sleep(INDEXER_SHOOT_TIME); // tempo maior para garantir giro completo
            indexer.setPower(0);
            temBola = false;
            return;
        }

        if (!temBola) indexer.setPower(0.7);
        else indexer.setPower(0);
    }
    public void setShooterPowerLow(double pooi){
        shooterD.setPower(pooi);
    }


    // ------------------- AUXILIARES -----------------------------

    public void stopAll() {
        rotationMotorX.setPower(0);
        shooterD.setPower(0);
    }

    public void aimAndShoot() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            //alignToTarget(result);
            controlShooterPower(result);
        } else {
            stopAll();
        }
    }
}
