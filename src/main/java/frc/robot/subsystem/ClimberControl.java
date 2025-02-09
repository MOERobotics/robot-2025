package frc.robot.subsystem;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberControl extends Subsystem {

    @AutoLog
     class ClimberInputs{
        boolean canGoUp;
        boolean canGoDown;

        AngularVelocity motorVelocity;

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


    default AngularVelocity getMotorVelocity(){
        return this.getSensors().motorVelocity;
    }



}
