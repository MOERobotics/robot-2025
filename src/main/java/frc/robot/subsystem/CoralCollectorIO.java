package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface CoralCollectorIO {

    @AutoLog
    public class CoralCollectorInputs{
        public boolean hasCoral;
        public AngularVelocity angularVelocityLeft;

        public AngularVelocity angularVelocityRight;

    }
      boolean hasCoral() ;
    public boolean inFrontReef() ;
    public void setCoralVelocity(AngularVelocity leftPower, AngularVelocity rightPower);

    public AngularVelocity getLeftPower();

    public AngularVelocity getRightPower();





}
