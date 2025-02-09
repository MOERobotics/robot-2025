package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class Climber extends MOESubsystem<ClimberInputsAutoLogged> implements ClimberControl  {
    SparkMax climberMotor;
    CANcoder climberEncoder;
    public Climber (
    SparkMax climberMotor, CANcoder climberEncoder
    ){
        this.setSensors(new ClimberInputsAutoLogged());
        this.climberMotor = climberMotor;
        this.climberEncoder = climberEncoder;
    }

    @Override
    public void setClimberVelocity(AngularVelocity power) {
        climberMotor.set(power.in(RadiansPerSecond));
    }

    @Override
    public void readSensors(ClimberInputsAutoLogged sensors) {
        sensors.position = climberEncoder.getAbsolutePosition().getValue();
        sensors.motorVelocity = RPM.of(climberMotor.getEncoder().getVelocity());
        sensors.canGoUp = true;
        sensors.canGoDown = true;
    }
}