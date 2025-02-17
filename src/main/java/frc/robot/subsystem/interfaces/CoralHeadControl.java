package frc.robot.subsystem.interfaces;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystem.interfaces.CoralHeadInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

public interface CoralHeadControl extends Subsystem {

    @AutoLog
    public class CoralHeadInputs {
        public boolean hasCoral;
        public boolean inFrontReef;
        public AngularVelocity velocityLeft;
        public AngularVelocity velocityRight;

    }
    CoralHeadInputsAutoLogged getSensors();

    default boolean hasCoral(){
        return this.getSensors().hasCoral;
    }

    default boolean inFrontReef(){
        return this.getSensors().inFrontReef;
    }

    public void setCoralVelocity(AngularVelocity leftAngularVelocity, AngularVelocity rightAngularVelocity);

    default AngularVelocity getLeftVelocity(){
        return this.getSensors().velocityLeft;
    }

    default AngularVelocity getRightVelocity(){
        return this.getSensors().velocityRight;
    }
}
