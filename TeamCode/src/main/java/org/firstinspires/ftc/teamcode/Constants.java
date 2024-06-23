package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.SENSE.JOYSTICK_SENSE;
import static org.firstinspires.ftc.teamcode.Constants.SENSE.TRIGGER_SENSE;

public class Constants {
    public static class BASE {
        public static final double COUNTS_PER_MOTOR_REV = 560; // Encoder resolution
        public static final double DRIVE_GEAR_REDUCTION = 1.0; // No external gearing
        public static final double WHEEL_DIAMETER_INCHES = 3.54;
        public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
        public static final double ABOUT_TO_MAKE_CONTACT_DISTANCE = 15; // cm

        // (Optional) Distance sensor with Kalman filter
        public static final double K_Q = 0.3; // High values put more emphasis on the sensor.
        public static final double K_R = 3; // High Values put more emphasis on regression.
        public static final int K_N = 3; // The number of estimates in the past we perform regression on.
    }

    public static class SPEED {
        public static final double AUTO_MAX_SPEED = 95.0 / 100.0;
        public static final double TELE_MAX_SPEED = 0.6;
        public static final double BASE_SPEED = 0.6;
        public static final double LOCKER_ELEVATOR_SPEED = 0.3;
        public static final double LIFT_SPEED = 0.5; // TODO: tune
        public static final double CLIMBER_SPEED = 0.5; // TODO: tune
    }

    public static class SENSE {
        public static final double JOYSTICK_SENSE = 0.05;
        public static final float TRIGGER_SENSE = 0.2f;
    }
    public static class CLIMB {
        public static final double CLIMBER_MIN_SPEED = 0; // tick per sec

        // TODO: Change this from climb ticks to convert into ticks from inches
        public static final int CLIMBER_MIN_TICKS = 2965;

        // TODO: Measure max current when it gets stuck
        public static final double CLIMBER_MAX_CURRENT = 5;
    }

}
