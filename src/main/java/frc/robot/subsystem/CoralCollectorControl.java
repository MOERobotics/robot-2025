package frc.robot.subsystem;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.MOESubsystem;
import org.littletonrobotics.junction.AutoLog;

public interface CoralCollectorControl extends Subsystem {

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
