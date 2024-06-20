package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveController;
import org.firstinspires.ftc.teamcode.Subsystems.TankDrive;

import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;
@Autonomous
public class Auto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    TankDrive drive = new TankDrive(this);
    AutoDriveSystem autoDriveSystem = new AutoDriveSystem(drive, runtime, this);
    DriveController driveController = new DriveController(this);


    @Override


    public void runOpMode() {
        waitForStart();
        runAuto();
        while (opModeIsActive()) {
            driveController.controlledDrive();
        }
    }

    public void runAuto() {
        autoDriveSystem.followLine();
        //Chan co khi???
        autoDriveSystem.encoderDrive(BASE_SPEED, 2,2,4);
    }
}
