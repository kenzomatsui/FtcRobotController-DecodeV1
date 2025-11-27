package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensor{
    DistanceSensor sensorDistance;
    DcMotor indexer;
    double DISTANCIA_BOLA = 138;
    public Sensor (HardwareMap hardwareMap) {
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        indexer = hardwareMap.get(DcMotor.class, "index");
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
        public void sensorFunciona(){
            double dist = sensorDistance.getDistance(DistanceUnit.MM);

            if (dist < DISTANCIA_BOLA){
                indexer.setPower(0);
            }else{
                indexer.setPower(0.7);
            }
        }
    }
