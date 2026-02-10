package org.firstinspires.ftc.teamcode.drive.unused.camera;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.objects.ShooterObjBlue;

@Disabled
@TeleOp(name = "ShooterPOO", group = "Drive")
public class Shooter extends LinearOpMode {

    private ShooterObjBlue shooter; // Objeto da classe Shooter

    @Override
    public void runOpMode() {
        shooter = new ShooterObjBlue(hardwareMap); // Inicializa o sistema

        telemetry.addData("Status", "Inicializado");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {


            telemetry.update();
        }
    }
}