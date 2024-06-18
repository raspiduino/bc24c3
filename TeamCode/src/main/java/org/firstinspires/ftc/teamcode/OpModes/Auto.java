package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDrive;

@Autonomous
public class Auto extends LinearOpMode {
    private Drivebase drive;
    private final ElapsedTime runtime = new ElapsedTime();
    private AutoDrive autoDrive;

    @Override
    public void runOpMode() {
        drive = new Drivebase(hardwareMap);
        autoDrive = new AutoDrive(drive, runtime, this);

        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
