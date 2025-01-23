package frc.robot.subsystem;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MOESubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class AlgaeCollector extends MOESubsystem<AlgaeCollectorInputsAutoLogged>{
    AlgaeCollectorIO IO;

    public AlgaeCollector(AlgaeCollectorIO IO){
        this.IO = IO;
        this.setSensors(new AlgaeCollectorInputsAutoLogged());
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public void setCollectorVelocity(AngularVelocity collectorVelocity){
        IO.setCollectorVelocity(collectorVelocity);
    }

    public void setWheelVelocity(AngularVelocity wheelVelocity){
        IO.setWheelVelocity(wheelVelocity);
    }

    public void stopWheel(){
        IO.setWheelVelocity(RotationsPerSecond.of(0));
    }

    public boolean hasAlgae(){
        return IO.hasAlgae();
    }

    public boolean inStartPosition(){
        return IO.inStartPosition();
    }

    public boolean inCollectPosition(){
        return IO.inCollectPosition();
    }
}
