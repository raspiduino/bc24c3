package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Climber extends DoubleMotorStructure {
    public Climber(LinearOpMode opMode) {
        init(opMode, "leftClimber", "rightClimer");
    }
}
