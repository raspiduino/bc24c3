package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSystem;
//import org.firstinspires.ftc.teamcode.Subsystems.DriveController;
import org.firstinspires.ftc.teamcode.Subsystems.TankDrive;

import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;
import static org.firstinspires.ftc.teamcode.Constants.CLIMB.*;

@Autonomous
public class Auto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    TankDrive drive = new TankDrive(this);
    AutoDriveSystem autoDriveSystem = new AutoDriveSystem(drive, runtime, this);
    //DriveController driveController = new DriveController(this);
    Climber climber = new Climber(this);

    @Override
    public void runOpMode() {
        waitForStart();

        // Follow line
        autoDriveSystem.followLine();

        // Line reached, let's climb
        climber.setSpeed(CLIMBER_SPEED, CLIMBER_SPEED);
        sleep(500);

        // Wait for climber to reach destination
        while (opModeIsActive() && climber.getCurrent() < CLIMBER_MAX_CURRENT && climber.getPos() < CLIMBER_MIN_TICKS) {}

        // Stop when got there
        climber.setSpeed(0, 0);

        // Move forward
        autoDriveSystem.encoderDrive(BASE_SPEED, 2,2,4);

        /*while (opModeIsActive()) {
            driveController.controlledDrive();
        }*/
    }
}
