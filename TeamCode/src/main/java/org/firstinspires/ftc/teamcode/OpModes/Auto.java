package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSystem;
import org.firstinspires.ftc.teamcode.Subsystems.TankDrive;

import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;
@Autonomous
public class Auto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        TankDrive drive = new TankDrive(this);
        AutoDriveSystem autoDriveSystem = new AutoDriveSystem(drive, runtime, this);
        Climber climber = new Climber(this);
        waitForStart();
        //Move from the start area (Red area) to the autonomous area
//        autoDriveSystem.encoderDrive(BASE_SPEED, 2, 2, 2);
//        while(!drive.aboutToMakeContact()) {
//            autoDriveSystem.followLine();
//        }
//        climber.setPos(CLIMB_TICKS, CLIMB_TICKS);
//        sleep(5000);
//        autoDriveSystem.encoderDrive(BASE_SPEED, 6, 6, 3);
        while (opModeIsActive()) {
            continue;
        }
    }
}
