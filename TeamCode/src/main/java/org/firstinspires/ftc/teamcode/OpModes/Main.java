package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;

@TeleOp(name = "main")
public class Main extends LinearOpMode {
    private double SENSE = 0.05;
    
    private Drivebase drive;

    private double sense(double val) {
        return (Math.abs(val) >= SENSE) ? val : 0;
    }

    public void runOpMode() {
        drive = new Drivebase(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            double ly = sense(-gamepad1.left_stick_y);
            double rx = sense(gamepad1.right_stick_x);
            drive.setPowerToDrive(ly, rx);
        }
    }
}
