package org.firstinspires.ftc.teamcode.Subsystems;
import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Drivebase {
    public DcMotor          leftMotor       = null;
    public DcMotor          rightMotor      = null;
    public ColorSensor      colorSensor     = null;
    public DistanceSensor   distanceSensor  = null;
    public BNO055IMU        imu             = null;

    private double robotHeading  = 0;
    private double headingOffset = 0;
    public Drivebase(HardwareMap hardwareMap) {
        leftMotor       = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor      = hardwareMap.get(DcMotor.class, "rightMotor");
        colorSensor    = hardwareMap.get(ColorSensor.class, "colorsensor");
        distanceSensor  = hardwareMap.get(DistanceSensor.class, "distancesensor");
        imu             = hardwareMap.get(BNO055IMU.class, "imu");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Get rid of drifting in auto???
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPowerToDrive(double ly, double rx) {
        double MAXSPEED = 0.6;
        double max = Math.abs(ly) + Math.abs(rx);
        max = Math.max(max, 1);
        double pLeft = (ly + rx) / max;
        double pRight = (ly - rx) / max;
        leftMotor.setPower(pLeft*MAXSPEED);
        rightMotor.setPower((pRight*MAXSPEED));
    }

    public boolean isWhite(ColorSensor colorSensor) {
        double red = colorSensor.red();
        double green = colorSensor.green();
        double blue = colorSensor.blue();
        double color = 0.2126*red + 0.7152*green + 0.0722*blue;
        return color > 128;
    }

    public boolean aboutToMakeContact() {
        return distanceSensor.getDistance(DistanceUnit.CM) <= 10;
    }
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
}
