package frc.robot.subsystem;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface CoralCollectorControl {

    @AutoLog
    public class CoralCollectorInputs{
        public boolean hasCoral;
        public AngularVelocity velocityLeft;

        public AngularVelocity velocityRight;

    }
    public boolean hasCoral() ;

    public boolean inFrontReef() ;
    public void setCoralVelocity(AngularVelocity leftPower, AngularVelocity rightPower);

    public AngularVelocity getLeftVelocity();

    public AngularVelocity getRightVelocity();





}
