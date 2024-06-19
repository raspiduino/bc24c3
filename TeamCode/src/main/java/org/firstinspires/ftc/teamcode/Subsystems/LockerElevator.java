package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class LockerElevator extends DoubleMotorStructure {
    public LockerElevator (LinearOpMode opMode) {
        init(opMode, "leftElevator", "rightElevator");
    }
}
