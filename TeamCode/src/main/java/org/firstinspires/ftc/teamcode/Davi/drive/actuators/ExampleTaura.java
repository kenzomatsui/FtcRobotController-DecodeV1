package org.firstinspires.ftc.teamcode.Davi.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Davi.drive.objects.TauraServo;

@TeleOp(name="Taura Move", group="Linear OpMode")
public class ExampleTaura extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "servo");
        AnalogInput potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");

        TauraServo tauraServo = new TauraServo(servo);

        tauraServo.setAnalogFeedbackSensor(potentiometer);

        tauraServo.setPosition(0.0);

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()) {

            if(gamepad1.a)
                tauraServo.setPosition(1.0);
            else if(gamepad1.b)
                tauraServo.setPosition(0.5);
            else
                tauraServo.setPosition(0.0);

            telemetry.addData("Pos comandada", "%.3f", tauraServo.getPosition());
            telemetry.addData("Pos lida", "%.3f", tauraServo.getUniversalPosition());
            telemetry.addData("Pos graus absoluto", "%.3f", tauraServo.getRawPositionInDegrees());
            telemetry.addData("Pos graus incremental", "%.3f", tauraServo.getIncrementalPositionInDegrees());

            telemetry.update();
        }
    }

}