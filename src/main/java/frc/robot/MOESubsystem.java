package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class MOESubsystem<SensorType extends LoggableInputs> extends SubsystemBase {

    @Getter @Setter
    private SensorType sensors;
    public String inputsKey;

    public MOESubsystem(SensorType sensors) {
        this(sensors, sensors.getClass().getName());
    }
    public MOESubsystem(SensorType sensors, String inputsKey) {
        this.sensors = sensors;
        this.inputsKey = inputsKey;
        System.out.println("Constructed Subsystem type: " + getClass());
    }

    @Override
    public void periodic() {
        this.readSensors(this.sensors);
        Logger.processInputs(inputsKey, sensors);
    }

    public void readSensors(SensorType sensors) {}

}
