package frc.robot.subsystem;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorSubsystem {
    @AutoLog
    class ElevatorInputs{
        public Distance height;
        public Angle angle;
        public AngularVelocity horizontalSpeed;
        public LinearVelocity verticalSpeed;
    }

    public ElevatorInputsAutoLogged getSensors();
    public void moveVertically(LinearVelocity speed);

    public void moveHorizontally(AngularVelocity speed);

    default Distance getHeight() {
        return this.getSensors().height;
    }

    default Angle getAngle() {
        return this.getSensors().angle;
    }
}
