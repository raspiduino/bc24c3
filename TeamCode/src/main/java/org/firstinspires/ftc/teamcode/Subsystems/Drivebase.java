package org.firstinspires.ftc.teamcode.Subsystems;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Drivebase {
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public ColorSensor colorSensor1 = null;
    public ColorSensor colorSensor2 = null;
    public DistanceSensor distanceSensor = null;
    private IMU angle = null;
    public Drivebase(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        colorSensor1 = hardwareMap.get(ColorSensor.class, "colorsensor1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "colorsensor2");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distancesensor");

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

}
