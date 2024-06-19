package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DoubleMotorStructure {
    private DcMotorEx left;
    private DcMotorEx right;

    public void init(LinearOpMode opMode, String leftName, String rightName) {
        this.left = (DcMotorEx) opMode.hardwareMap.get(DcMotor.class, leftName);
        this.right = (DcMotorEx) opMode.hardwareMap.get(DcMotor.class, rightName);

        this.left.setDirection(DcMotor.Direction.REVERSE);

        this.left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSpeed(double left, double right) {
        this.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.left.setPower(left);
        this.right.setPower(right);
    }

    public double getAverSpeed() {
        return (this.left.getVelocity() + this.right.getVelocity()) / 2;
    }

    public double getCurrent() {
        return (this.left.getCurrent(CurrentUnit.AMPS) + this.right.getCurrent(CurrentUnit.AMPS)) / 2;
    }

    public void setPos(int left, int right) {
        this.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.left.setTargetPosition(left);
        this.right.setTargetPosition(right);
    }
}
