package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDrive;

public class Auto extends LinearOpMode {
    private Drivebase drive;
    private final ElapsedTime runtime = new ElapsedTime();
    private AutoDrive autoDrive;

    @Override
    public void runOpMode() {
        drive = new Drivebase(hardwareMap);
        autoDrive = new AutoDrive(drive, runtime, telemetry, this);

        waitForStart();

        // Example usage of the encoderDrive method
        autoDrive.encoderDrive(0.5, 10, 10, 5.0);
    }
}
