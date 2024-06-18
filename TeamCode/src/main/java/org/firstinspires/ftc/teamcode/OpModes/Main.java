package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;

import static org.firstinspires.ftc.teamcode.Constants.SENSE.*;

@TeleOp(name = "main")
public class Main extends LinearOpMode {
    private double sense(double val) {
        return (Math.abs(val) >= JOYSTICK_SENSE) ? val : 0;
    }

    public void runOpMode() {
        Drivebase drive = new Drivebase(hardwareMap);
        waitForStart();

        while(opModeIsActive()) {
            double ly = sense(-gamepad1.left_stick_y);
            double rx = sense(gamepad1.right_stick_x);
            drive.setPowerToDrive(ly, rx);
        }
    }
}
