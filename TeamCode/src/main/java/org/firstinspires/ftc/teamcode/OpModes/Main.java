package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
@TeleOp(name = "main")
public class Main extends LinearOpMode {
    private Drivebase drive;

    public void runOpMode() {
        drive = new Drivebase(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            drive.setPowerToDrive(ly, rx);
        }
    }
}
