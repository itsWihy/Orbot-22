package frc.lib.generic.motor;

import com.ctre.phoenix6.signals.GravityTypeValue;

public class MotorProperties {
    public enum IdleMode {
        COAST, BRAKE
    }

    /**
     * Enumeration of different control modes for a motor.
     */
    public enum ControlMode {
        /**
         * Control the motor output based on the desired current.
         * <p>Control type: Current control</p>
         * <p>Units: Amperes (A)</p>
         */
        CURRENT,

        /**
         * Control the motor output based on the desired voltage.
         * <p>Control type: Voltage control</p>
         * <p>Units: Volts (V)</p>
         */
        VOLTAGE,

        /**
         * Control the motor output based on the desired duty cycle.
         * <p>Control type: Duty cycle control</p>
         * <p>Units: Percentage (%)</p>
         * <p>Note: 0.5 represents 50% duty cycle.</p>
         */
        PERCENTAGE_OUTPUT,

        /**
         * Control the motor to achieve a specific position.
         * <p>Control type: Position control</p>
         * <p>Units: Rotations</p>
         */
        POSITION,

        /**
         * Control the motor to achieve a specific velocity.
         * <p>Control type: Velocity control</p>
         * <p>Units: Rotations per second (RPS)</p>
         */
        VELOCITY
    }

    public enum SparkType {
        MAX, FLEX
    }

    public static final class Slot {
        private final double kP, kD, kI, kV, kA, kS, kG;
        private final GravityTypeValue gravityType;

        public Slot(double kP, double kI, double kD, double kV, double kA, double kS, double kG, GravityTypeValue gravityType) {
            this.kP = kP;
            this.kD = kD;
            this.kI = kI;
            this.kV = kV;
            this.kA = kA;
            this.kS = kS;
            this.kG = kG;
            this.gravityType = gravityType;
        }

        public Slot(double kP, double kI, double kD, double kV, double kA, double kS) {
            this(kP, kI, kD, kV, kA, kS, 0, null);
        }

        public Slot(double kP, double kI, double kD) {
            this(kP, kI, kD, 0, 0, 0, 0, null);
        }

        public double kP() {
            return kP;
        }

        public double kD() {
            return kD;
        }

        public double kI() {
            return kI;
        }

        public double kV() {
            return kV;
        }

        public double kA() {
            return kA;
        }

        public double kS() {
            return kS;
        }

        public double kG() {
            return kG;
        }

        public GravityTypeValue gravityType() {
            return gravityType;
        }
    }
}
