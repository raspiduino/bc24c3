package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Lift extends DoubleMotorStructure {
    public Lift(LinearOpMode opMode) {
        init(opMode, "leftLift", "rightLift");
    }
}
