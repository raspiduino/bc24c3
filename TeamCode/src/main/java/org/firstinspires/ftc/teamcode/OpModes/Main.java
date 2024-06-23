package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveController;

@TeleOp(name = "Manual")
public class Main extends LinearOpMode {
    DriveController driveController = new DriveController(this);

    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            driveController.controlledDrive();
        }
    }
}
