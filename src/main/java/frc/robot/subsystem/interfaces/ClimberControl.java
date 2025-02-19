package frc.robot.subsystem.interfaces;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystem.interfaces.ClimberInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberControl extends Subsystem {

    @AutoLog
     class ClimberInputs{
        public boolean canGoUp;
        public boolean canGoDown;
        public AngularVelocity motorVelocity;
        public Angle position;
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
