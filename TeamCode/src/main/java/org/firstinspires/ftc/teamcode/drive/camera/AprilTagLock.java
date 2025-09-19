package org.firstinspires.ftc.teamcode.drive.camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class AprilTagLock extends LinearOpMode {
    private Limelight3A limelight;
    private DcMotor rotationMotorX = null; // Single motor for rotation X
    private DcMotor rotationMotorY = null; // Single motor for rotation Y
    public static double XKp = 0.02; // Proportional constant
    public static double XKi = 0.00; // Integral constant (start with 0, add if needed)
    public static double XKd = 0.001; // Derivative constant (start with small value)

    private static final double XTOLERANCE = 0.5;   // degrees, adjust as needed
    private static final double XMAX_POWER = 0.5;   // Maximum motor power to apply

    // PID variables
    private double Xintegral = 0;
    private double XlastError = 0;
    private long XlastTime = 0;


    public static double YKp = 0.02; // Proportional constant
    public static double YKi = 0.00; // Integral constant (start with 0, add if needed)
    public static double YKd = 0.003; // Derivative constant (start with small value)

    private static final double YTOLERANCE = 0.5;   // degrees, adjust as needed
    private static final double YMAX_POWER = 0.5;   // Maximum motor power to apply

    // PID variables
    private double Yintegral = 0;
    private double YlastError = 0;
    private long YlastTime = 0;

    public void runOpMode(){

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Set how often we ask Limelight for data (100 times per second)
        limelight.start(); // Tell Limelight to start looking!
        limelight.pipelineSwitch(7);

        rotationMotorX = hardwareMap.get(DcMotor.class, "RMX"); // Adjust name as per your robot's configuration
        rotationMotorY = hardwareMap.get(DcMotor.class, "RMY"); // Adjust name as per your robot's configuration

        rotationMotorX.setDirection(DcMotor.Direction.FORWARD);
        rotationMotorY.setDirection(DcMotor.Direction.FORWARD);

        rotationMotorX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotorY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        XlastTime = System.currentTimeMillis();
        YlastTime = System.currentTimeMillis();

        while (opModeIsActive()){

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()){
                double tx = result.getTx();
                double ty = result.getTy();

                double Xerror = -tx;
                double Yerror = -ty;

                long currentTime = System.currentTimeMillis();
                double deltaTime = (currentTime - YlastTime) / 1000.0;

                Xintegral += Xerror * deltaTime;
                Yintegral += Yerror * deltaTime;
                double Xderivative = (Xerror - XlastError) / deltaTime;
                double Yderivative = (Yerror - YlastError) / deltaTime;

                double Xpower = XKp * Xerror + XKi * Xintegral + XKd * Xderivative;
                double Ypower = YKp * Yerror + YKi * Yintegral + YKd * Yderivative;

                Xpower = Math.max(-XMAX_POWER, Math.min(Xpower, XMAX_POWER));
                Ypower = Math.max(-YMAX_POWER, Math.min(Ypower, YMAX_POWER));

                if (Math.abs(Xerror) > XTOLERANCE) {
                    rotationMotorX.setPower(Xpower);
                    telemetry.addData("Status", "Adjusting");
                } else {
                    rotationMotorX.setPower(0);
                    telemetry.addData("Status", "Alinhado!");
                    Xintegral = 0; // Reset integral when aligned to prevent wind-up
                }
                if (Math.abs(Yerror) > YTOLERANCE) {
                    rotationMotorY.setPower(Ypower);
                    telemetry.addData("Status", "Adjusting");
                } else {
                    rotationMotorY.setPower(0);
                    telemetry.addData("Status", "Alinhado!");
                    Yintegral = 0; // Reset integral when aligned to prevent wind-up
                }
                telemetry.addData("Target X", tx);
                telemetry.addData("Error", Xerror);
                telemetry.addData("Power", Xpower);
                telemetry.addData("Target X", ty);
                telemetry.addData("Error", Yerror);
                telemetry.addData("Power", Ypower);

                XlastError = Xerror;
                XlastTime = currentTime;
                YlastError = Yerror;
                YlastTime = currentTime;

            } else {
                // No target visible, stop motor and reset PID
                rotationMotorX.setPower(0);
                Xintegral = 0;
                XlastError = 0;
                rotationMotorY.setPower(0);
                Yintegral = 0;
                YlastError = 0;
                telemetry.addData("Status", "No Targets");
            }
            telemetry.update();
        }
    }
}

