package frc.robot.subsystem.interfaces;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface CoralHeadControl extends Subsystem {

    @AutoLog
    class CoralHeadInputs {
        public boolean hasCoral;
        public boolean inFrontReef;
        public double leftPower;
        public double rightPower;
        public AngularVelocity velocityLeft = RPM.zero();
        public AngularVelocity velocityRight = RPM.zero();
        public Voltage leftAppliedVolts = Volts.zero();
        public Voltage rightAppliedVolts = Volts.zero();

    }
    CoralHeadInputsAutoLogged getSensors();

    default boolean hasCoral(){
        return this.getSensors().hasCoral;
    }

    default boolean inFrontReef(){
        return this.getSensors().inFrontReef;
    }

    void setCoralVelocity(AngularVelocity leftAngularVelocity, AngularVelocity rightAngularVelocity);

    default AngularVelocity getLeftVelocity(){
        return this.getSensors().velocityLeft;
    }

    default AngularVelocity getRightVelocity(){
        return this.getSensors().velocityRight;
    }
}
