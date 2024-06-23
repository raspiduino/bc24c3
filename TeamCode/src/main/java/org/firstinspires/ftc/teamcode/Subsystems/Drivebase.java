package org.firstinspires.ftc.teamcode.Subsystems;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.checkerframework.checker.units.qual.K;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.Constants.BASE.*;

public class Drivebase {
    public DcMotorEx        leftMotor;
    public DcMotorEx        rightMotor;
    public ColorSensor      colorSensor;
    public DistanceSensor   distanceSensor;
    public IMU              imu;
    public boolean          filterDistanceSensor;

    private KalmanFilter filter = new KalmanFilter(K_Q, K_R, K_N);

    public void init(LinearOpMode opMode,
                String leftMotor,
                String rightMotor,
                String colorSensor,
                String distanceSensor,
                String imu,
                boolean filterDistanceSensor) {
        this.leftMotor       = (DcMotorEx)      opMode.hardwareMap.get(DcMotorEx.class, leftMotor);
        this.rightMotor      = (DcMotorEx)      opMode.hardwareMap.get(DcMotorEx.class, rightMotor);
        this.colorSensor     = (ColorSensor)    opMode.hardwareMap.get(ColorSensor.class, colorSensor);
        this.distanceSensor  = (DistanceSensor) opMode.hardwareMap.get(DistanceSensor.class, distanceSensor);
        this.imu             = (IMU)            opMode.hardwareMap.get(IMU.class, imu);
        this.filterDistanceSensor = filterDistanceSensor;

        // Set motors' direction and mode
        this.leftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        this.leftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.rightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setPowerToDrive(double ly, double rx, double max_speed) {
        double max = Math.max(Math.abs(ly) + Math.abs(rx), 1);
        double pLeft = (ly + rx) / max;
        double pRight = (ly - rx) / max;
        this.leftMotor.setPower(pLeft * max_speed);
        this.rightMotor.setPower(pRight * max_speed);
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
        double rawReading = this.distanceSensor.getDistance(DistanceUnit.CM);
        if (!filterDistanceSensor) {
            return rawReading <= ABOUT_TO_MAKE_CONTACT_DISTANCE;
        }

        return filter.estimate(rawReading) <= ABOUT_TO_MAKE_CONTACT_DISTANCE;
    }
    public double getHeading() {
        Orientation angles = this.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle * 180.0 / Math.PI;
    }
    public double inchesToTicks(double cm) {
        return (cm / 2.54) * COUNTS_PER_INCH;
    }
}
