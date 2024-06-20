package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.LockerElevator;
import org.firstinspires.ftc.teamcode.Subsystems.TankDrive;

import static org.firstinspires.ftc.teamcode.Constants.SENSE.*;
import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;

public class DriveController {
    private final LinearOpMode opMode;
    private TankDrive drive;
    private  LockerElevator lockerElevator;

    public DriveController(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public double sense(double val) {
        return (Math.abs(val) >= JOYSTICK_SENSE) ? val : 0;
    }

    public boolean trigger(float val) {
        return (Math.abs(val) >= TRIGGER_SENSE);
    }

    public void controlledDrive() {
        // Base
        double ly = sense(-opMode.gamepad1.left_stick_y);
        double rx = sense(opMode.gamepad1.right_stick_x);
        drive.setPowerToDrive(ly, rx, TELE_MAX_SPEED);

        // Locker & Elevator
        if (opMode.gamepad1.left_bumper) {
            lockerElevator.setSpeed(LOCKER_ELEVATOR_SPEED, LOCKER_ELEVATOR_SPEED);
        } else if (trigger(opMode.gamepad1.left_trigger)) {
            lockerElevator.setSpeed(-LOCKER_ELEVATOR_SPEED, -LOCKER_ELEVATOR_SPEED);
        } else {
            lockerElevator.setSpeed(0, 0);
        }

        // Lift
        if (opMode.gamepad1.right_bumper) {
            lockerElevator.setSpeed(LIFT_SPEED, LIFT_SPEED);
        } else if (trigger(opMode.gamepad1.right_trigger)) {
            lockerElevator.setSpeed(-LIFT_SPEED, -LIFT_SPEED);
        } else {
            lockerElevator.setSpeed(0, 0);
        }
    }
}
