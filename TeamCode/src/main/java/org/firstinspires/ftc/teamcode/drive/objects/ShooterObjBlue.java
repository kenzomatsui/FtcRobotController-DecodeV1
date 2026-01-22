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
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterObjBlue {
    public NormalizedColorSensor colorSensor;
    public View relativeLayout;

    private DcMotorEx indexer, intake;

    public ShooterObjBlue(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        indexer = hardwareMap.get(DcMotorEx.class, "index");
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

    // ------------------- ALINHAMENTO ---------------------------

    //public double get_power() {return power;}
    public void SHOOTER3(boolean go) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        if (go) {
            while (timer.seconds() < 2.0) {
                indexer.setPower(1);
                intake.setPower(-1);
            }
            indexer.setPower(0.6);
        }
    }

    public void testMotor(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        if (colors.green > 0.010){
            indexer.setPower(0);
        } else {
            indexer.setPower(0.6);
        }
    }
    public void Shoot(double trigger){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        if (colors.green > 0.010){
            indexer.setPower(0);
        } else {
            indexer.setPower(0.7);
        }
        if (trigger>0.3){
            indexer.setPower(1);
        }
    }
}