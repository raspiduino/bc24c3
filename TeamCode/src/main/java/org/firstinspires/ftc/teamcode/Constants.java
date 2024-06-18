package org.firstinspires.ftc.teamcode;

public class Constants {
    public static class BASE {
        public static final double COUNTS_PER_MOTOR_REV = 560; // Encoder resolution
        public static final double DRIVE_GEAR_REDUCTION = 1.0; // No external gearing
        public static final double WHEEL_DIAMETER_INCHES = 3.54;
        public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    }

    public static class SPEED {
        public static final double AUTO_MAX_SPEED = 95.0 / 100.0;
        public static final double TELE_MAX_SPEED = 0.6;
    }

    public static class SENSE {
        public static final double JOYSTICK_SENSE = 0.05;
    }
}
