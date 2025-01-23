package frc.robot.subsystem;

import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.*;

public class FakeCoralCollector extends MOESubsystem<CoralCollectorInputsAutoLogged> implements CoralCollectorIO  {

    public FakeCoralCollector(){
        this.setSensors(new CoralCollectorInputsAutoLogged());
    }

    @Override
    public boolean hasCoral() {

        return false;
    }

    @Override
    public boolean inFrontReef() {
        return false;
    }

    @Override
    public void setCoralVelocity(AngularVelocity leftPower, AngularVelocity rightPower) {

    }

    @Override
    public AngularVelocity getLeftVelocity() {
        return null;

    }

    @Override
    public AngularVelocity getRightVelocity() {
        return null;
    }
}
