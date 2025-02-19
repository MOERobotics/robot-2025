package frc.robot.subsystem.fakes;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.ClimberInputsAutoLogged;
import frc.robot.subsystem.interfaces.ClimberControl;

public class FakeClimber extends MOESubsystem<ClimberInputsAutoLogged> implements ClimberControl {

    public FakeClimber(ClimberInputsAutoLogged sensors) {
        super(new ClimberInputsAutoLogged());
    }

    @Override
    public void setClimberVelocity(AngularVelocity power) {
    }

    @Override
    public void readSensors(ClimberInputsAutoLogged sensors) {
        super.readSensors(sensors);
    }
}
