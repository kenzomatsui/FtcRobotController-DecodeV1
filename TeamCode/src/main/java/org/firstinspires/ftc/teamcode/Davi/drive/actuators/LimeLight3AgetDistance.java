package org.firstinspires.ftc.teamcode.Davi.drive.actuators;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp
public class LimeLight3AgetDistance extends OpMode {
    private Limelight3A limelight;
    double x;
    double y;
    double z;

    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(9);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
    }

    public void start(){limelight.start();}

    public void loop(){
        telemetry.addData("Status: ", "running...");
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                // Certifique-se de que a pipeline da Limelight está configurada para AprilTags e 3D
                // e que a AprilTag está sendo detectada.

                Pose3D tagPoseCameraSpace = fr.getTargetPoseCameraSpace();

                if (tagPoseCameraSpace != null) {
                    x = tagPoseCameraSpace.getPosition().x; // Deslocamento lateral em metros
                    y = tagPoseCameraSpace.getPosition().y; // Deslocamento vertical em metros
                    z = tagPoseCameraSpace.getPosition().z; // Distância frontal em metros

                    // Calcular a distância em linha reta (Euclidiana 3D) em metros
                    double dwll_meters = (Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2)));

                    telemetry.addData("dwll (metros)", "%.2f", dwll_meters);
                }

            }
        }

    }

    public double getLLX(){
        return x;
    }
    public double getLLY(){
        return y;
    }
}
