package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Drivebase {
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private ColorSensor colorSensor = null;
    private DistanceSensor distanceSensor = null;

    private IMU angle = null;
    public Drivebase(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorsensor");
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
        double max = Math.abs(ly) + Math.abs(rx);
        max = Math.max(max, 1);
        double pLeft = (ly + rx) / max;
        double pRight = (ly - rx) / max;
        leftMotor.setPower(pLeft);
        rightMotor.setPower((pRight));
    }

    public boolean isWhite() {
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
