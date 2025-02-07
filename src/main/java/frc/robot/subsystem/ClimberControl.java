package frc.robot.subsystem;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberControl extends Subsystem {

    @AutoLog
     class ClimberInputs{
        boolean canGoUpTop;
        boolean canGoDownTop;

        boolean canGoUpBottom;
        boolean canGoDownBottom;
        AngularVelocity topMotorVelocity;
        AngularVelocity bottomMotorVelocity;

        Angle topPosition;
        Angle bottomPosition;
    }


    ClimberInputsAutoLogged getSensors();

    default boolean canGoUpTop(){
        return this.getSensors().canGoUpTop;
    }

    default boolean canGoDownTop(){
        return this.getSensors().canGoDownTop;
    }

    default boolean canGoUpBottom(){
        return this.getSensors().canGoUpBottom;
    }

    default boolean canGoDownBottom(){
        return this.getSensors().canGoDownBottom;
    }


    public void setClimberTopVelocity(AngularVelocity power);

    public void setClimberBottomVelocity(AngularVelocity power);


    public default Angle getTopPosition(){ return this.getSensors().topPosition; }

    public default Angle getBottomPosition(){ return this.getSensors().bottomPosition; }

    default AngularVelocity getTopMotorVelocity(){
        return this.getSensors().topMotorVelocity;
    }

    default AngularVelocity getBottomMotorVelocity(){
        return this.getSensors().bottomMotorVelocity;
    }


}
