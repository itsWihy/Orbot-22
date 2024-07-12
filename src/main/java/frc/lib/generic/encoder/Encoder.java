package frc.lib.generic.encoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

public interface Encoder {
    /** Returns the encoder position, in Rotations*/
    double getEncoderPosition();
    double getEncoderVelocity();

    /** Signals are lazily loaded - only these explicity called will be updated. Thus you must call this method. when using a signal.*/
    void setSignalUpdateFrequency(EncoderSignal signal);

    StatusSignal<Double> getRawStatusSignal(EncoderSignal signal);

    /**
     * Refreshes all status signals.
     * This has the same effect as calling {@link com.ctre.phoenix6.BaseStatusSignal#refreshAll(BaseStatusSignal...)}}}.
     * DO NOT USE if not necessary.
     */
    void refreshStatusSignals(EncoderSignal... signals);

    boolean configure(EncoderConfiguration encoderConfiguration);
}
