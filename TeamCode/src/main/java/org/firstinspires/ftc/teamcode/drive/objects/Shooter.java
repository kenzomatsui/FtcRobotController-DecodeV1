package org.firstinspires.ftc.teamcode.drive.objects;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Shooter{

    public static double distancia = 110;
    private DistanceSensor sensorDistance;
    private DcMotor Indexer;

    private Limelight3A limelight;
    private DcMotor rotationMotorX = null; // Motor da Base giratória
    private DcMotor shooterD = null; // Motor do shooter da direita
    private DcMotor shooterE = null; // Motor do shooter da esquerda


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

    public Shooter(HardwareMap hardwareMap) {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Set how often we ask Limelight for data (100 times per second)
        limelight.start(); // Tell Limelight to start looking!
        limelight.pipelineSwitch(7);

        rotationMotorX = hardwareMap.get(DcMotor.class, "RMX"); // Adjust name as per your robot's configuration
        rotationMotorX.setDirection(DcMotor.Direction.FORWARD);
        rotationMotorX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterD = hardwareMap.get(DcMotor.class, "shootD"); // Ajuste o nome
        shooterD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterD.setDirection(DcMotor.Direction.FORWARD);
        shooterE = hardwareMap.get(DcMotor.class, "shootE"); // Ajuste o nome
        shooterE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterE.setDirection(DcMotor.Direction.REVERSE);

        // you can use this as a regular DistanceSensor.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        Indexer = hardwareMap.get(DcMotor.class, "index");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
    }
    public void Shoot(double power){

    }
}
