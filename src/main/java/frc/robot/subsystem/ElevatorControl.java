package frc.robot.subsystem;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorControl extends Subsystem {
    @AutoLog
    class ElevatorInputs{
        public Distance extension;
        public Angle angle;
        public AngularVelocity horizontalSpeed;
        public LinearVelocity extensionSpeed;
    }

    public ElevatorInputsAutoLogged getSensors();

    public void moveVertically(LinearVelocity speed);

    public void moveHorizontally(AngularVelocity speed);

    default Distance getExtension() {
        return this.getSensors().extension;
    }

    default Angle getAngle() {
        return this.getSensors().angle;
    }
}
