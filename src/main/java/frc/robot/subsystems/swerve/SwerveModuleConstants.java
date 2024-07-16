package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.encoder.*;
import frc.lib.generic.motor.*;
import frc.lib.generic.simulation.SimulationProperties;

import static frc.lib.generic.motor.MotorSignal.SignalType.*;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_GEAR_RATIO;
import static frc.robot.subsystems.swerve.SwerveConstants.STEER_GEAR_RATIO;

public class SwerveModuleConstants {
    static final MotorConfiguration steerMotorConfiguration = new MotorConfiguration();
    static final MotorConfiguration driveMotorConfiguration = new MotorConfiguration();

    static final MotorProperties.IdleMode ANGLE_NEUTRAL_MODE = MotorProperties.IdleMode.BRAKE;
    static final MotorProperties.IdleMode DRIVE_NEUTRAL_MODE = MotorProperties.IdleMode.BRAKE;

    static final double OPEN_LOOP_RAMP = 0.1;
    static final double CLOSED_LOOP_RAMP = 0.0;

    static final boolean CAN_CODER_INVERT = false;
    static final boolean ANGLE_MOTOR_INVERT = true;
    static final boolean DRIVE_MOTOR_INVERT = false;

    static final int ANGLE_CURRENT_LIMIT = 30;
    static final int DRIVE_SUPPLY_CURRENT_LIMIT = 35;
    static final int DRIVE_STATOR_CURRENT_LIMIT = 50;

    static final EncoderSignal STEER_POSITION_SIGNAL = new EncoderSignal(EncoderSignal.SignalType.POSITION, true);
    static final MotorSignal DRIVE_POSITION_SIGNAL = new MotorSignal(POSITION, true);

    static final MotorProperties.Slot DRIVE_SLOT = new MotorProperties.Slot(
            0.053067, 0.0, 0.0,
            0.10861,
            0.023132,
            0.27053);

    protected static final Motor
            FL_STEER_MOTOR = MotorFactory.createSpark("FL_STEER_MOTOR", 11, MotorProperties.SparkType.MAX),
            FR_STEER_MOTOR = MotorFactory.createSpark("FR_STEER_MOTOR", 10, MotorProperties.SparkType.MAX),
            RL_STEER_MOTOR = MotorFactory.createSpark("RL_STEER_MOTOR", 6, MotorProperties.SparkType.MAX),
            RR_STEER_MOTOR = MotorFactory.createSpark("RR_STEER_MOTOR", 9, MotorProperties.SparkType.MAX);

    protected static final Motor
            FL_DRIVE_MOTOR = MotorFactory.createTalonFX("FL_DRIVE_MOTOR", 14),
            FR_DRIVE_MOTOR = MotorFactory.createTalonFX("FR_DRIVE_MOTOR", 13),
            RL_DRIVE_MOTOR = MotorFactory.createTalonFX("RL_DRIVE_MOTOR", 13),
            RR_DRIVE_MOTOR = MotorFactory.createTalonFX("RR_DRIVE_MOTOR", 12);

    protected static final Encoder
            FL_STEER_ENCODER = EncoderFactory.createCanCoder("FL_STEER_ENCODER", 18),
            FR_STEER_ENCODER = EncoderFactory.createCanCoder("FR_STEER_ENCODER", 20),
            RL_STEER_ENCODER = EncoderFactory.createCanCoder("RL_STEER_ENCODER", 19),
            RR_STEER_ENCODER = EncoderFactory.createCanCoder("RR_STEER_ENCODER", 21);

    static final double[] STEER_ENCODER_OFFSET = {0.677246, 0.282715, 0.533447, 0.313721};

    static final Encoder[] STEER_ENCODER = {FL_STEER_ENCODER, FR_STEER_ENCODER, RL_STEER_ENCODER, RR_STEER_ENCODER};
    static final Motor[] STEER_MOTOR = {FL_STEER_MOTOR, FR_STEER_MOTOR, RL_STEER_MOTOR, RR_STEER_MOTOR};
    static final Motor[] DRIVE_MOTOR = {FL_DRIVE_MOTOR, FR_DRIVE_MOTOR, RL_DRIVE_MOTOR, RR_DRIVE_MOTOR};


    static {
        configureSteerConfiguration();
        configureDriveConfiguration();

        for (int i = 0; i < 4; i++) {
            configureSteerEncoder(STEER_ENCODER[i], Rotation2d.fromRotations(STEER_ENCODER_OFFSET[i]));
            configureDriveMotor(DRIVE_MOTOR[i]);
            configureSteerMotor(STEER_MOTOR[i], STEER_ENCODER[i]);
        }
    }

    protected static final SwerveModule[] MODULES = new SwerveModule[]{
            new SwerveModule(FL_DRIVE_MOTOR, FL_STEER_MOTOR, FL_STEER_ENCODER),
            new SwerveModule(FR_DRIVE_MOTOR, FR_STEER_MOTOR, FR_STEER_ENCODER),
            new SwerveModule(RL_DRIVE_MOTOR, RL_STEER_MOTOR, RL_STEER_ENCODER),
            new SwerveModule(RR_DRIVE_MOTOR, RR_STEER_MOTOR, RR_STEER_ENCODER)
    };

    private static void configureSteerEncoder(Encoder steerEncoder, Rotation2d angleOffset) {
        EncoderConfiguration encoderConfiguration = new EncoderConfiguration();

        encoderConfiguration.invert = CAN_CODER_INVERT;
        encoderConfiguration.sensorRange = EncoderProperties.SensorRange.ZeroToOne;
        encoderConfiguration.offsetRotations = -angleOffset.getRotations();

        steerEncoder.configure(encoderConfiguration);

        steerEncoder.setSignalUpdateFrequency(STEER_POSITION_SIGNAL);
    }

    private static void configureDriveMotor(Motor driveMotor) {
        driveMotor.setupSignalsUpdates(DRIVE_POSITION_SIGNAL);

        driveMotor.setupSignalsUpdates(new MotorSignal(VELOCITY));
        driveMotor.setupSignalsUpdates(new MotorSignal(VOLTAGE));
        driveMotor.setupSignalsUpdates(new MotorSignal(TEMPERATURE));

        driveMotor.configure(driveMotorConfiguration);
    }

    private static void configureSteerMotor(Motor steerMotor, Encoder encoder) {
        steerMotor.setupSignalsUpdates(new MotorSignal(POSITION));
        steerMotor.configure(steerMotorConfiguration);

        steerMotor.setExternalPositionSupplier(encoder::getEncoderPosition);
    }

    private static void configureDriveConfiguration() {
        driveMotorConfiguration.idleMode = DRIVE_NEUTRAL_MODE;
        driveMotorConfiguration.inverted = DRIVE_MOTOR_INVERT;

        driveMotorConfiguration.gearRatio = DRIVE_GEAR_RATIO;

        driveMotorConfiguration.statorCurrentLimit = DRIVE_STATOR_CURRENT_LIMIT;
        driveMotorConfiguration.supplyCurrentLimit = DRIVE_SUPPLY_CURRENT_LIMIT;

        driveMotorConfiguration.slot0 = DRIVE_SLOT;

        driveMotorConfiguration.dutyCycleOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        driveMotorConfiguration.dutyCycleClosedLoopRampPeriod = CLOSED_LOOP_RAMP;

        driveMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.SIMPLE_MOTOR, DCMotor.getFalcon500(1), STEER_GEAR_RATIO,0.03);
        driveMotorConfiguration.simulationSlot = new MotorProperties.Slot(50, 0, 0);
    }

    private static void configureSteerConfiguration() {
        steerMotorConfiguration.slot0 = new MotorProperties.Slot(7, 0, 0, 0, 0, 0);

        steerMotorConfiguration.supplyCurrentLimit = ANGLE_CURRENT_LIMIT;
        steerMotorConfiguration.inverted = ANGLE_MOTOR_INVERT;
        steerMotorConfiguration.idleMode = ANGLE_NEUTRAL_MODE;

        steerMotorConfiguration.gearRatio = STEER_GEAR_RATIO;

        steerMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.SIMPLE_MOTOR, DCMotor.getNEO(1), STEER_GEAR_RATIO,0.03);
        steerMotorConfiguration.simulationSlot = new MotorProperties.Slot(50, 0, 0);

        steerMotorConfiguration.closedLoopContinuousWrap = true;
    }
}
