package frc.robot.subsystem.interfaces;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ClimberControl extends Subsystem {

    @AutoLog
    class ClimberInputs {
        public boolean canGoUp;
        public boolean canGoDown;
        public double motorPower;
        public AngularVelocity motorVelocity = RPM.zero();
        public Voltage motorAppliedVolts = Volts.zero();
        public Angle position = Degrees.zero();
    }

    ClimberInputsAutoLogged getSensors();



    default boolean canGoUp() {
        return this.getSensors().canGoUp;
    }

    default boolean canGoDown() {
        return this.getSensors().canGoDown;
    }

    void setClimberVelocity(AngularVelocity power);

    default Angle getPosition() {
        return this.getSensors().position;
    }

    default AngularVelocity getMotorVelocity() {
        return this.getSensors().motorVelocity;
    }

}
