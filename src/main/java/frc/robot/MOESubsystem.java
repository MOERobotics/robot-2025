package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class MOESubsystem<SensorType extends LoggableInputs> extends SubsystemBase {

    @Getter @Setter
    private SensorType sensors;

    public MOESubsystem() {
        System.out.println("Constructed Subsystem type: " + getClass());
    }

    @Override
    public void periodic() {
        this.readSensors(this.sensors);
        Logger.processInputs(getName()/*sensors.getClass().getSimpleName()*/, sensors);
     //   this.visionMeasurement();
       // Logger.processInputs("Vision Pose",visionMeasurement());
    }

    public void readSensors(SensorType sensors) {}

}
