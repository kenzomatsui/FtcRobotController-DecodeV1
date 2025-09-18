package org.firstinspires.ftc.teamcode.drive.camera;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class AprilTagLock extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor rotationMotor = null; // Single motor for rotation

    // PID Constants - These will need to be tuned for your specific robot
    public static double Kp = 0.02; // Proportional constant
    public static double Ki = 0.00; // Integral constant (start with 0, add if needed)
    public static double Kd = 0.001; // Derivative constant (start with small value)

    private static final double TOLERANCE = 0.5;   // degrees, adjust as needed
    private static final double MAX_POWER = 0.5;   // Maximum motor power to apply

    // PID variables
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = 0;

    @Override
    public void runOpMode() {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Set how often we ask Limelight for data (100 times per second)
        limelight.start(); // Tell Limelight to start looking!
        limelight.pipelineSwitch(7);

        // Initialize the single rotation motor
        rotationMotor = hardwareMap.get(DcMotor.class, "RM"); // Adjust name as per your robot's configuration

        // Set motor direction if needed (e.g., if positive power turns it the wrong way)
        rotationMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set motor mode
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        lastTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx(); // Horizontal Offset From Crosshair To Target (degrees)

                double error = -tx; // Error is negative of tx for aligning to center (0 degrees)

                long currentTime = System.currentTimeMillis();
                double deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds

                // Calculate PID components
                integral += error * deltaTime;
                double derivative = (error - lastError) / deltaTime;

                // Calculate total power
                double power = Kp * error + Ki * integral + Kd * derivative;

                // Clamp power to max_power
                power = Math.max(-MAX_POWER, Math.min(power, MAX_POWER));

                if (Math.abs(error) > TOLERANCE) {
                    rotationMotor.setPower(power);
                    telemetry.addData("Status", "Adjusting");
                } else {
                    rotationMotor.setPower(0);
                    telemetry.addData("Status", "Alinhado!");
                    integral = 0; // Reset integral when aligned to prevent wind-up
                }
                telemetry.addData("Target X", tx);
                telemetry.addData("Error", error);
                telemetry.addData("Power", power);

                lastError = error;
                lastTime = currentTime;

            } else {
                // No target visible, stop motor and reset PID
                rotationMotor.setPower(0);
                telemetry.addData("Status", "No Targets");
                integral = 0;
                lastError = 0;
            }
            telemetry.update();
        }
    }
}