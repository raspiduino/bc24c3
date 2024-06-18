package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class AutoDrive {
    private final Drivebase drive;
    private final ElapsedTime runtime;
    private final Telemetry telemetry;
    private final LinearOpMode opMode;

    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.54;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public AutoDrive(Drivebase drive, ElapsedTime runtime, Telemetry telemetry, LinearOpMode opMode) {
        this.drive = drive;
        this.runtime = runtime;
        this.telemetry = telemetry;
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
            drive.leftMotor.setPower(Math.abs(speed) * 95 / 100);
            drive.rightMotor.setPower(Math.abs(speed) * 95 / 100);

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
}
