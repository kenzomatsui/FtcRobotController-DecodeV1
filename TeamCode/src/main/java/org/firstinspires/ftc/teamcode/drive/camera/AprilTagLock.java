package org.firstinspires.ftc.teamcode.drive.camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class AprilTagLock extends OpMode {
    Limelight3A limelight;
    IMU imu;
    DcMotor motor;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Isso define com que frequência pedimos dados ao Limelight (100 vezes por segundo)
        limelight.start(); // Isso diz ao Limelight para começar a procurar!
        limelight.pipelineSwitch(7); // Mudar para o pipeline número 7

        // Mapeia a IMU do arquivo de configuração de hardware do robô.
        imu = hardwareMap.get(IMU.class, "imu");
        // Define os parâmetros de inicialização da IMU, especificando a orientação física do hub Rev no robô.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));
        // Inicializa a IMU com os parâmetros definidos.
        imu.initialize(parameters);

        //Declaração do HM do motor da plataforma
        motor = hardwareMap.get(DcMotor.class,"motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void loop(){

        LLResult result = limelight.getLatestResult();
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(robotYaw);
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            double tx = result.getTx();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x * 1000;
                double y = botpose_mt2.getPosition().y * 1000;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");

             }
        }
    }
}

