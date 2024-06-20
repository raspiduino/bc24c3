package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.LockerElevator;
import org.firstinspires.ftc.teamcode.Subsystems.TankDrive;

import static org.firstinspires.ftc.teamcode.Constants.SENSE.*;
import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;

@TeleOp(name = "main")
public class Main extends LinearOpMode {
    TankDrive drive = new TankDrive(this);
    LockerElevator lockerElevator = new LockerElevator(this);

    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            runTeleOp();
        }
    }

    public double sense(double val) {
        return (Math.abs(val) >= JOYSTICK_SENSE) ? val : 0;
    }

    public boolean trigger(float val) {
        return (Math.abs(val) >= TRIGGER_SENSE);
    }

    private void runTeleOp() {
        // Base
        double ly = sense(-gamepad1.left_stick_y);
        double rx = sense(gamepad1.right_stick_x);
        drive.setPowerToDrive(ly, rx, TELE_MAX_SPEED);

        // Locker & Elevator
        if (gamepad1.left_bumper) {
            lockerElevator.setSpeed(LOCKER_ELEVATOR_SPEED, LOCKER_ELEVATOR_SPEED);
        } else if (trigger(gamepad1.left_trigger)) {
            lockerElevator.setSpeed(-LOCKER_ELEVATOR_SPEED, -LOCKER_ELEVATOR_SPEED);
        } else {
            lockerElevator.setSpeed(0, 0);
        }

        // Lift
        if (gamepad1.right_bumper) {
            lockerElevator.setSpeed(LIFT_SPEED, LIFT_SPEED);
        } else if (trigger(gamepad1.right_trigger)) {
            lockerElevator.setSpeed(-LIFT_SPEED, -LIFT_SPEED);
        } else {
            lockerElevator.setSpeed(0, 0);
        }
    }
}
