package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.MOESubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberControl extends Subsystem {

    @AutoLog
    public class ClimberInputs{
        boolean canGoUp;
        boolean canGoDown;
        AngularVelocity velocity;
        Angle position;

    }


    ClimberInputsAutoLogged getSensors();

    default boolean canGoUp(){
        return this.getSensors().canGoUp;
    }

    default boolean canGoDown(){
        return this.getSensors().canGoDown;
    }

    public void setClimberVelocity(AngularVelocity power);

    public default Angle getPosition(){ return this.getSensors().position; }


    default AngularVelocity getClimberVelocity(){
        return this.getSensors().velocity;
    }




}
