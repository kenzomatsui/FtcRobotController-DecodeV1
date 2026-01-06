package org.firstinspires.ftc.teamcode.drive.objects;

import static android.os.SystemClock.sleep;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class ShooterObjBlue {


    private static final double KP = 0.06, KI = 0.0, KD = 0.0025;
    private static final double TOLERANCIA = 0.5, XMAX_POWER = 0.5;

    private static final double MIN_POWER = 0.4, MAX_POWER = 1.0;
    private static final double TARGET_TA = 5.0, SCALE_FACTOR = 0.08;

    private Limelight3A limelight;
    public NormalizedColorSensor colorSensor;
    public View relativeLayout;

    private DcMotorEx indexer, rotationMotorX, shooterD, intake;

    private double integral = 0, lastError = 0;
    private long lastTime = System.currentTimeMillis();

    public ShooterObjBlue(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(7);

        rotationMotorX = hardwareMap.get(DcMotorEx.class, "RMX");
        rotationMotorX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor");

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        try {
            runSample();
        } finally {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }
    }
    protected void runSample() {
        float gain = 2;

        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        final float[] hsvValues = new float[3];

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
            colorSensor.setGain(gain);

            Color.colorToHSV(colors.toColor(), hsvValues);

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
                }
            });
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
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        if (colors.green > 0.010){
            indexer.setPower(0);
        } else {
            indexer.setPower(1);
        }
    }
    public void Shoot(double trigger){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        if (colors.green > 0.010){
            indexer.setPower(0);
        } else {
            indexer.setPower(1);
        }
        if (trigger>0.1){
            indexer.setPower(1);
        }
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
            alignToTarget(result);
            controlShooterPower(result);
        } else {
            stopAll();
        }
    }
}
