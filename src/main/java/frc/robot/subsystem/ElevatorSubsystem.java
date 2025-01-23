package frc.robot.subsystem;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;

public interface ElevatorSubsystem {
    public void moveVertically(LinearVelocity speed);

    public void moveHorizontally(AngularVelocity speed);

    public Distance getHeight();

    public Angle getAngle();
}
