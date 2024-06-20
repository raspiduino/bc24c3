package org.firstinspires.ftc.teamcode.Subsystems;
//This code was copied shamelessly from the FTC samples
//org/firstinspires/ftc/robotcontroller/external/samples/RobotAutoDriveByEncoder_Linear.java
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;

public class EncoderDrive extends Drivebase {
    private final Drivebase drive;
    private final ElapsedTime runtime;
    private final Telemetry telemetry;
    private final LinearOpMode opMode;

    public EncoderDrive(Drivebase drive, ElapsedTime runtime, LinearOpMode opMode) {
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
            newLeftTarget = drive.leftMotor.getCurrentPosition() + (int) inchesToTicks(leftInches);
            newRightTarget = drive.rightMotor.getCurrentPosition() + (int) inchesToTicks(rightInches);
            drive.leftMotor.setTargetPosition(newLeftTarget);
            drive.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            drive.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the timeout time and start motion.
            runtime.reset();
            drive.leftMotor.setPower(Math.abs(speed) * AUTO_MAX_SPEED);
            drive.rightMotor.setPower(Math.abs(speed) * AUTO_MAX_SPEED);
            drive.setPowerToDrive(Math.abs(speed), Math.abs(speed), AUTO_MAX_SPEED);

            // Keep looping while we are still active, and there is time left, and both motors are running.
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

            opMode.sleep(250);   // Optional pause after each move.
        }
    }
}
