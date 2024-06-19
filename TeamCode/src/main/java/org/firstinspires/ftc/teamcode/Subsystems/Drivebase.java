package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;

public class Drivebase {
    public DcMotor          leftMotor       = null;
    public DcMotor          rightMotor      = null;
    public ColorSensor      colorSensor     = null;
    public DistanceSensor   distanceSensor  = null;
    public IMU              imu             = null;

    public Drivebase(HardwareMap hardwareMap) {
        leftMotor       = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor      = hardwareMap.get(DcMotor.class, "rightMotor");
        colorSensor     = hardwareMap.get(ColorSensor.class, "colorsensor");
        distanceSensor  = hardwareMap.get(DistanceSensor.class, "distancesensor");
        imu             = hardwareMap.get(IMU.class, "imu");

        // Set motors' direction and mode
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPowerToDrive(double ly, double rx) {
        double max = Math.max(Math.abs(ly) + Math.abs(rx), 1);
        double pLeft = (ly + rx) / max;
        double pRight = (ly - rx) / max;
        leftMotor.setPower(pLeft * TELE_MAX_SPEED);
        rightMotor.setPower(pRight * TELE_MAX_SPEED);
    }

    public boolean isWhite(ColorSensor colorSensor) {
        double red = colorSensor.red();
        double green = colorSensor.green();
        double blue = colorSensor.blue();
        double color = 0.2126 * red + 0.7152 * green + 0.0722 * blue;
        return color > 128;
    }
    //TODO: Use Kalman Filter to effectively predict if the robot has made contacted or not
    public boolean aboutToMakeContact() {
        return distanceSensor.getDistance(DistanceUnit.CM) <= 15;
    }

    public double getHeading() {
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle * 180.0 / Math.PI;
    }
}
