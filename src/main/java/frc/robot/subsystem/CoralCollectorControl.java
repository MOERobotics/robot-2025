package frc.robot.subsystem;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.MOESubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

public interface CoralCollectorControl extends Subsystem {

    @AutoLog
    public class CoralCollectorInputs{
        boolean hasCoral;
        boolean inFrontReef;
        AngularVelocity velocityLeft;
        AngularVelocity velocityRight;

    }
    CoralCollectorInputsAutoLogged getSensors();

    default boolean hasCoral(){
        return this.getSensors().hasCoral;
    }

    default boolean inFrontReef(){
        return this.getSensors().inFrontReef;
    }

    public void setCoralVelocity(AngularVelocity leftPower, AngularVelocity rightPower);

    default AngularVelocity getLeftVelocity(){
        return this.getSensors().velocityLeft;
    }

    default AngularVelocity getRightVelocity(){
        return this.getSensors().velocityRight;
    }
}
