package org.firstinspires.ftc.teamcode.drive.camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp
public class Shooter extends LinearOpMode {

    public static double distancia = 110;
    private DistanceSensor sensorDistance;
    private DcMotor Motor;

    private Limelight3A limelight;
    private DcMotor rotationMotorX = null; // Single motor for rotation X
    private DcMotor driveMotor = null; // Motor que vai acelerar conforme a distância


    // PID Constants - These will need to be tuned for your specific robot
    public static double Kp = 0.02; // Proportional constant
    public static double Ki = 0.00; // Integral constant (start with 0, add if needed)
    public static double Kd = 0.001; // Derivative constant (start with small value)

    private static final double TOLERANCE = 0.5;   // degrees, adjust as needed
    private static final double XMAX_POWER = 0.5;   // Maximum motor power to apply

    // PID variables
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = 0;
    public static double MIN_POWER = 0.3;   // Potência mínima
    public static double MAX_POWER = 1;   // Potência máxima
    public static double TARGET_TA = 5.0;   // "Área" esperada quando estiver na distância ideal
    public static double SCALE_FACTOR = 0.064; // Ajuste da curva de resposta

    @Override
    public void runOpMode() {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Set how often we ask Limelight for data (100 times per second)
        limelight.start(); // Tell Limelight to start looking!
        limelight.pipelineSwitch(7);

        // Initialize the single rotation motor
        rotationMotorX = hardwareMap.get(DcMotor.class, "RMX"); // Adjust name as per your robot's configuration

        // Set motor direction if needed (e.g., if positive power turns it the wrong way)
        rotationMotorX.setDirection(DcMotor.Direction.FORWARD);

        // Set motor mode
        rotationMotorX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveMotor = hardwareMap.get(DcMotor.class, "RMTa"); // Ajuste o nome
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setDirection(DcMotor.Direction.FORWARD);

        // you can use this as a regular DistanceSensor.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        Motor = hardwareMap.get(DcMotor.class, "index");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();

        lastTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            boolean isDetected = false;

            if(sensorDistance.getDistance(DistanceUnit.MM) < distancia){
                isDetected = true;
                Motor.setPower(0);
            }else{
                Motor.setPower(0.5);
            }
            if (gamepad1.a){
                Motor.setPower(1);
            }

            telemetry.addData("Bola identificada: ", isDetected);
            // generic DistanceSensor methods.
            telemetry.addData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx(); // Horizontal Offset From Crosshair To Target (degrees)

                double error = tx; // Error is negative of tx for aligning to center (0 degrees)

                long currentTime = System.currentTimeMillis();
                double deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds

                // Calculate PID components
                integral += error * deltaTime;
                double derivative = (error - lastError) / deltaTime;

                // Calculate total power
                double Xpower = Kp * error + Ki * integral + Kd * derivative;

                // Clamp power to max_power
                Xpower = (Math.max(-XMAX_POWER, Math.min(Xpower, XMAX_POWER))) * 2;

                double ta = result.getTa(); // Área da AprilTag (relacionada à distância)

                // Calcula o erro relativo à área desejada
                double distanceError = TARGET_TA - ta;

                // Quanto menor a área (mais longe), maior o erro => mais potência
                double power = MIN_POWER + (distanceError * SCALE_FACTOR);

                // Limita entre os valores mínimos e máximos
                power = Math.max(MIN_POWER, Math.min(power, MAX_POWER));

                if (Math.abs(error) > TOLERANCE) {
                    rotationMotorX.setPower(Xpower);
                    telemetry.addData("Status", "Adjusting");
                } else {
                    rotationMotorX.setPower(0);
                    telemetry.addData("Status", "Alinhado!");
                    integral = 0; // Reset integral when aligned to prevent wind-up
                }
                telemetry.addData("Target X", tx);
                telemetry.addData("Error", error);
                telemetry.addData("Power", Xpower);

                lastError = error;
                lastTime = currentTime;
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
                rotationMotorX.setPower(0);
                telemetry.addData("Status", "No Targets");
                integral = 0;
                lastError = 0;
                driveMotor.setPower(0);
            }
            telemetry.update();
        }
    }
}
