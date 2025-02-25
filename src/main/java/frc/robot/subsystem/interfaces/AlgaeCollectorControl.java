package frc.robot.subsystem.interfaces;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface AlgaeCollectorControl extends Subsystem {
    @AutoLog
    class AlgaeCollectorInputs {
        public double wheelPower;
        public double armPower;
        public Voltage wheelAppliedVolts = Volts.zero();
        public AngularVelocity wheelVelocity = RPM.zero();
        public Voltage algaeArmAppliedVolts = Volts.zero();
        public Angle algaeArmAngle = Degrees.zero();
        public boolean hasAlgae;
        public boolean inStartPosition;
        public boolean inCollectPosition;
        public WheelState wheelState = WheelState.EJECTING;
        public AngularVelocity algaeArmVelocity = RadiansPerSecond.zero();

    }

    AlgaeCollectorInputsAutoLogged getSensors();

    default void setArmVelocity(AngularVelocity armVelocity) {
    }

    default void setWheelVelocity(AngularVelocity wheelVelocity) {
    }

    default Angle getArmAngle() {
        return this.getSensors().algaeArmAngle;
    }

    default boolean inStartPosition() {
        return this.getSensors().inStartPosition;
    }

    default boolean inCollectPosition() {
        return this.getSensors().inCollectPosition;
    }

    enum WheelState {
        COLLECTING,
        EJECTING,
        HOLDING
    }

    default boolean hasAlgae() {
        return this.getSensors().hasAlgae;
    }

    default WheelState getWheelstate() {
        return this.getSensors().wheelState;
    }

    void setWheelState(WheelState wheelState);

}
