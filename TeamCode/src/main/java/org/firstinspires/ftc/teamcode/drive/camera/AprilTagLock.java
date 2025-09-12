package org.firstinspires.ftc.teamcode.drive.camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class AprilTagLock extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    // Motor control constants
    private static final double DRIVE_SPEED = 0.2; // Adjust as needed
    private static final double TURN_SPEED = 0.1;  // Adjust as needed
    private static final double TOLERANCE = 0.5;   // degrees, adjust as needed

    @Override
    public void runOpMode() {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Set how often we ask Limelight for data (100 times per second)
        limelight.start(); // Tell Limelight to start looking!

        // Initialize motors (adjust names as per your robot's configuration)
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        // Most robots will need to reverse one motor
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set motor modes
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx(); // Horizontal Offset From Crosshair To Target (degrees)
                // double ty = result.getTy(); // Vertical Offset From Crosshair To Target (degrees) - Not used for horizontal alignment

                double error = -tx; // Error is negative of tx for aligning to center (0 degrees)

                if (Math.abs(error) > TOLERANCE) {
                    // Proportional control for turning
                    double turn = error * TURN_SPEED; // Simple proportional control
                    leftMotor.setPower(turn);
                    rightMotor.setPower(-turn);
                    telemetry.addData("Status", "Turning");
                } else {
                    // Aligned horizontally, stop turning
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    telemetry.addData("Status", "Alinhado!");
                }
                telemetry.addData("Target X", tx);
                telemetry.addData("Error", error);
            } else {
                // No target visible, stop motors
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                telemetry.addData("Status", "No Targets");
            }
            telemetry.update();
        }
    }
}


