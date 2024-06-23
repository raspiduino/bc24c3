package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TankDrive extends Drivebase {
    public TankDrive(LinearOpMode opMode) {
        init(opMode, "leftMotor", "rightMotor", "colorSensor", "distanceSensor", "imu", false);
    }

    public TankDrive(LinearOpMode opMode, boolean filterDistanceSensor) {
        init(opMode, "leftMotor", "rightMotor", "colorSensor", "distanceSensor", "imu", filterDistanceSensor);
    }
}
