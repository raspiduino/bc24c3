package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Constants.BASE.*;
import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;

public class AutoDrive {
    private final Drivebase drive;
    private final ElapsedTime runtime;
    private final Telemetry telemetry;
    private final LinearOpMode opMode;

    public AutoDrive(Drivebase drive, ElapsedTime runtime, LinearOpMode opMode) {
        this.drive = drive;
        this.runtime = runtime;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = drive.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = drive.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            drive.leftMotor.setTargetPosition(newLeftTarget);
            drive.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            drive.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            drive.leftMotor.setPower(Math.abs(speed) * AUTO_MAX_SPEED);
            drive.rightMotor.setPower(Math.abs(speed) * AUTO_MAX_SPEED);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opMode.opModeIsActive() && (runtime.seconds() < timeoutS) && (drive.leftMotor.isBusy() && drive.rightMotor.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d", drive.leftMotor.getCurrentPosition(), drive.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            drive.leftMotor.setPower(0);
            drive.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            drive.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            opMode.sleep(250);   // optional pause after each move.
        }
    }
    public void followLine() {
        double heading = drive.getHeading();
        //If Color Sensor still can detect the white line then keep going
        if (drive.isWhite(drive.colorSensor)) {
            encoderDrive(BASE_SPEED, 2, 2, 1);
        } else {
            /*
            The white line is not detected, now we use IMU to find if the robot oversteered to the left or to the right
            The heading is reading data coming from the IMU if the value is 0 < x < 180 then it turned left and -180 < x < 0 is to the right
            */
            if(heading < 0) {
                encoderDrive(BASE_SPEED, 1, 2, 1);
            } else if (heading > 0) {
                encoderDrive(BASE_SPEED, 2, 1, 1);
            } else {
                encoderDrive(BASE_SPEED,1 , 1, 0.5);
                //TODO: A corretion function to get the robot come back to the white line after placed horizontal wrong
            }
        }
    }
}
