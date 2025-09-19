package org.firstinspires.ftc.teamcode.drive.camera;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class PIDX extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor rotationMotorX = null; // Single motor for rotation X

    // PID Constants - These will need to be tuned for your specific robot
    public static double XKp = 0.02; // Proportional constant
    public static double XKi = 0.00; // Integral constant (start with 0, add if needed)
    public static double XKd = 0.001; // Derivative constant (start with small value)

    private static final double XTOLERANCE = 0.5;   // degrees, adjust as needed
    private static final double XMAX_POWER = 0.5;   // Maximum motor power to apply

    // PID variables
    private double Xintegral = 0;
    private double XlastError = 0;
    private long XlastTime = 0;

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        XlastTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx(); // Horizontal Offset From Crosshair To Target (degrees)

                double Xerror = -tx; // Error is negative of tx for aligning to center (0 degrees)

                long currentTime = System.currentTimeMillis();
                double deltaTime = (currentTime - XlastTime) / 1000.0; // Convert to seconds

                // Calculate PID components
                Xintegral += Xerror * deltaTime;
                double Xderivative = (Xerror - XlastError) / deltaTime;

                // Calculate total power
                double Xpower = XKp * Xerror + XKi * Xintegral + XKd * Xderivative;

                // Clamp power to max_power
                Xpower = Math.max(-XMAX_POWER, Math.min(Xpower, XMAX_POWER));

                if (Math.abs(Xerror) > XTOLERANCE) {
                    rotationMotorX.setPower(Xpower);
                    telemetry.addData("Status", "Adjusting");
                } else {
                    rotationMotorX.setPower(0);
                    telemetry.addData("Status", "Alinhado!");
                    Xintegral = 0; // Reset integral when aligned to prevent wind-up
                }
                telemetry.addData("Target X", tx);
                telemetry.addData("Error", Xerror);
                telemetry.addData("Power", Xpower);

                XlastError = Xerror;
                XlastTime = currentTime;

            } else {
                // No target visible, stop motor and reset PID
                rotationMotorX.setPower(0);
                telemetry.addData("Status", "No Targets");
                Xintegral = 0;
                XlastError = 0;
            }
            telemetry.update();
        }
    }
}