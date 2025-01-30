package frc.robot.subsystem;

import edu.wpi.first.units.measure.*;
import frc.robot.MOESubsystem;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeCollectorControl {
    @AutoLog
    public class AlgaeCollectorInputs{
         Voltage wheelAppliedVolts;
         AngularVelocity wheelVelocity;
         Voltage algaeCollectorAppliedVolts;
         Angle algaeCollectorAngle;
    }
    default void processInputs(AlgaeCollectorInputs inputs){};

    default void setCollectorVelocity(AngularVelocity collectorVelocity){};

    default void setWheelVelocity(AngularVelocity wheelVelocity){}

    boolean inStartPosition();

    boolean inCollectPosition();

    enum wheelState{
        COLLECTING,
        EJECTING,
        HOLDING
    };

    boolean hasAlgae();

}
