package org.firstinspires.ftc.teamcode.Davi.drive.actuators;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class KalmanFilterTest extends OpMode {
    private Follower follower;
    private Limelight3A limelight;
    LimeLight3AgetDistance LL = new LimeLight3AgetDistance();
    private final ElapsedTime timer = new ElapsedTime();
    private final Pose startPose = new Pose(72, 72, Math.toRadians(0)); //pose pro inicio do teleop azul com o intake vira pra spike mark



    public void init(){
        follower = Constants.createFollower(hardwareMap);

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

        follower.update();

        Pose currentPose = follower.getPose();
        LLResult result = limelight.getLatestResult();

        double maxConfidencePP = 1.0;
        double minConfidencePP = 0.2;
        double decayTime = 20.0;

        double confidencePP = maxConfidencePP - (timer.seconds() / decayTime);
        confidencePP = Math.max(confidencePP, minConfidencePP);

        double confidenceLL = 0;

        if (result != null && result.isValid()) {
            timer.reset();
            confidenceLL = 1.0;
        }

        double total = confidencePP + confidenceLL;
        if (total <= 0.0001) total = 1;

        double weightPP = confidencePP / total;
        double weightLL = confidenceLL / total;

        double finalX = (currentPose.getX() * weightPP) + (LL.getLLX() * weightLL);
        double finalY = (currentPose.getY() * weightPP) + (LL.getLLY() * weightLL);
        double finalW = Math.sqrt(finalY * finalY + finalX * finalX);

        telemetry.addData("Final X", finalX);
        telemetry.addData("Final Y", finalY);
        telemetry.addData("Final W", finalW);
        telemetry.update();
    }
}
