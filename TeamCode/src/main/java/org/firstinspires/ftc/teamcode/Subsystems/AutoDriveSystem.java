package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;

public class AutoDriveSystem extends Drivebase {
    private final Drivebase drive;
    private final ElapsedTime runtime;
    private final Telemetry telemetry;
    private final LinearOpMode opMode;
    private final EncoderDrive encoderDrive;

    public AutoDriveSystem(Drivebase drive, ElapsedTime runtime, LinearOpMode opMode) {
        this.drive = drive;
        this.runtime = runtime;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
        this.encoderDrive = new EncoderDrive(drive, runtime, opMode);
    }

    public void followLine() {
        double heading = drive.getHeading();
        // If Color Sensor still can detect the white line then keep going
        if (drive.isWhite(drive.colorSensor)) {
            encoderDrive.encoderDrive(BASE_SPEED, 2, 2, 1);
        } else {
            /* The white line is not detected, now we use IMU to find if the robot oversteered to the left or to the right
            The heading is reading data coming from the IMU if the value is 0 < x < 180 then it turned left and -180 < x < 0 is to the right */
            if (heading < 0) {
                encoderDrive.encoderDrive(BASE_SPEED, 1, 2, 1);
            } else if (heading > 0) {
                encoderDrive.encoderDrive(BASE_SPEED, 2, 1, 1);
            } else {
                encoderDrive.encoderDrive(BASE_SPEED, 1, 1, 0.5);
                // TODO: A correction function to get the robot come back to the white line after placed horizontal wrong
            }
        }
    }
}
