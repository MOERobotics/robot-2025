package frc.robot.subsystem;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.MOESubsystem;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeCollectorControl extends Subsystem {
    @AutoLog
    public class AlgaeCollectorInputs{
         Voltage wheelAppliedVolts;
         AngularVelocity wheelVelocity;
         Voltage algaeArmAppliedVolts;
         Angle algaeArmAngle;
         boolean hasAlgae;
         boolean inStartPosition;
         boolean inCollectPosition;
         WheelState wheelState;

    }
    AlgaeCollectorInputsAutoLogged getSensors();

    default void setArmVelocity(AngularVelocity armVelocity){}

    default void setWheelVelocity(AngularVelocity wheelVelocity){}

    default Angle getArmAngle(){
        return this.getSensors().algaeArmAngle;
    }

    default boolean inStartPosition(){
        return this.getSensors().inStartPosition;
    }

    default boolean inCollectPosition(){
        return this.getSensors().inCollectPosition;
    }

    enum WheelState {
        COLLECTING,
        EJECTING,
        HOLDING
    }

    default boolean hasAlgae(){
        return this.getSensors().hasAlgae;
    }

    default WheelState getWheelstate(){
        return this.getSensors().wheelState;
    }

    public void setWheelState(WheelState wheelState);

}
