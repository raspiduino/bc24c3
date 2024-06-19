package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDrive;

import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;
@Autonomous
public class Auto extends LinearOpMode {
    private Drivebase drive;
    private final ElapsedTime runtime = new ElapsedTime();
    private AutoDrive autoDrive;
    @Override
    public void runOpMode() {
        drive = new Drivebase(hardwareMap);
        autoDrive = new AutoDrive(drive, runtime, this);
        Climber climber = new Climber(this);
        waitForStart();
        //Move from the start area (Red area) to the autonomous area
        autoDrive.encoderDrive(BASE_SPEED, 2, 2, 2);
        while(!drive.aboutToMakeContact()) {
            autoDrive.followLine();
        }
        climber.setPos(CLIMB_TICKS, CLIMB_TICKS);
        sleep(5000);
        autoDrive.encoderDrive(BASE_SPEED, 6, 6, 3);
        while (opModeIsActive()) {
            continue;
        }
    }
}
