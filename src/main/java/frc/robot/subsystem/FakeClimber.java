package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.RPM;

public class FakeClimber extends MOESubsystem<ClimberInputsAutoLogged> implements ClimberControl {

    @Override
    public void setClimberVelocity(AngularVelocity power) {
    }

    @Override
    public void readSensors(ClimberInputsAutoLogged sensors) {
        super.readSensors(sensors);
    }
}
