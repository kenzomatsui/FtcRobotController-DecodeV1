package org.firstinspires.ftc.teamcode.drive.camera;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class Shooter extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor rotationMotorY = null; // Single motor for rotation X
    private DcMotor driveMotor = null; // Motor que vai acelerar conforme a distância


    // PID Constants - These will need to be tuned for your specific robot
    public static double YKp = 0.02; // Proportional constant
    public static double YKi = 0.00; // Integral constant (start with 0, add if needed)
    public static double YKd = 0.003; // Derivative constant (start with small value)

    private static final double YTOLERANCE = 0.5;   // degrees, adjust as needed
    private static final double YMAX_POWER = 0.5;   // Maximum motor power to apply

    // PID variables
    private double Yintegral = 0;
    private double YlastError = 0;
    private long YlastTime = 0;
    public static double MIN_POWER = 0.35;   // Potência mínima
    public static double MAX_POWER = 1;   // Potência máxima
    public static double TARGET_TA = 5.0;   // "Área" esperada quando estiver na distância ideal
    public static double SCALE_FACTOR = 0.084; // Ajuste da curva de resposta

    @Override
    public void runOpMode() {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Set how often we ask Limelight for data (100 times per second)
        limelight.start(); // Tell Limelight to start looking!
        limelight.pipelineSwitch(7);

        // Initialize the single rotation motor
        rotationMotorY = hardwareMap.get(DcMotor.class, "RMY"); // Adjust name as per your robot's configuration

        // Set motor direction if needed (e.g., if positive power turns it the wrong way)
        rotationMotorY.setDirection(DcMotor.Direction.FORWARD);

        // Set motor mode
        rotationMotorY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveMotor = hardwareMap.get(DcMotor.class, "RMTa"); // Ajuste o nome
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        YlastTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double ty = result.getTy(); // Horizontal Offset From Crosshair To Target (degrees)

                double Yerror = -ty; // Error is negative of tx for aligning to center (0 degrees)

                long currentTime = System.currentTimeMillis();
                double deltaTime = (currentTime - YlastTime) / 1000.0; // Convert to seconds

                // Calculate PID components
                Yintegral += Yerror * deltaTime;
                double Yderivative = (Yerror - YlastError) / deltaTime;

                // Calculate total power
                double Ypower = YKp * Yerror + YKi * Yintegral + YKd * Yderivative;

                // Clamp power to max_power
                Ypower = (Math.max(-YMAX_POWER, Math.min(Ypower, YMAX_POWER))) * 3;

                double ta = result.getTa(); // Área da AprilTag (relacionada à distância)

                // Calcula o erro relativo à área desejada
                double distanceError = TARGET_TA - ta;

                // Quanto menor a área (mais longe), maior o erro => mais potência
                double power = MIN_POWER + (distanceError * SCALE_FACTOR);

                // Limita entre os valores mínimos e máximos
                power = Math.max(MIN_POWER, Math.min(power, MAX_POWER));

                if (Math.abs(Yerror) > YTOLERANCE) {
                    rotationMotorY.setPower(Ypower);
                    telemetry.addData("Status", "Adjusting");
                } else {
                    rotationMotorY.setPower(0);
                    telemetry.addData("Status", "Alinhado!");
                    Yintegral = 0; // Reset integral when aligned to prevent wind-up
                }
                telemetry.addData("Target X", ty);
                telemetry.addData("Error", Yerror);
                telemetry.addData("Power", Ypower);

                YlastError = Yerror;
                YlastTime = currentTime;
                if (ta >= TARGET_TA) {
                    driveMotor.setPower(0);
                    telemetry.addData("Status", "Distância ideal ou muito perto");
                } else {
                    driveMotor.setPower(power);
                    telemetry.addData("Status", "Aproximando...");
                }
                telemetry.addData("Área (ta)", ta);
                telemetry.addData("Erro de distância", distanceError);
                telemetry.addData("Potência aplicada", power);

            } else {
                // No target visible, stop motor and reset PID
                rotationMotorY.setPower(0);
                telemetry.addData("Status", "No Targets");
                Yintegral = 0;
                YlastError = 0;
                driveMotor.setPower(0);
            }
            telemetry.update();
        }
    }
}