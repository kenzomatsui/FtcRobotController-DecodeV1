package org.firstinspires.ftc.teamcode.drive.camera;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
@Disabled
public class PIDY extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor rotationMotorY = null; // Single motor for rotation X

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

            } else {
                // No target visible, stop motor and reset PID
                rotationMotorY.setPower(0);
                telemetry.addData("Status", "No Targets");
                Yintegral = 0;
                YlastError = 0;
            }
            telemetry.update();
        }
    }
}