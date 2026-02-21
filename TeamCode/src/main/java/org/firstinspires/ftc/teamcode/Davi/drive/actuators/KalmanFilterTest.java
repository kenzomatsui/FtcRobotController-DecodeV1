package org.firstinspires.ftc.teamcode.Davi.drive.actuators;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp
public class KalmanFilterTest extends OpMode {
    private Follower follower;
    private Limelight3A limelight;
    public void init(){
        this.follower = follower;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
    }
    public void start(){
        limelight.start();

    }
    public void loop(){
        Pose currentPose = follower.getPose();

        LLResult result = limelight.getLatestResult();

        ElapsedTime timer = new ElapsedTime();

        double maxConfidencePP = 1.0;
        double minConfidencePP = 0.2;

        double confidencePP = 0;
        double confidenceLL = 0;
        double time = timer.seconds();

        // Decaimento linear do PP (ex: em 5 segundos ele cai pro mínimo)
        double decayTime = 30.0;

        confidencePP = maxConfidencePP - (time / decayTime);

        if (confidencePP < minConfidencePP) {
            confidencePP = minConfidencePP;
        }
        if (result.isValid()) {

            timer.reset();  // reseta o tempo

            confidenceLL = 1.0;
        }
        double total = confidencePP + confidenceLL;

        double weightPP = confidencePP / total;
        double weightLL = confidenceLL / total;

        //double finalDistance = ;
        double dxpp = currentPose.getX() * weightPP;
        double dypp = currentPose.getY() * weightPP;
        double dwpp = Math.sqrt(dypp * dypp + dxpp * dxpp);
        double dwLL = ;
        //double finalHeading = (ppHeading * weightPP) + (llHeading * weightLL);
    }
}
