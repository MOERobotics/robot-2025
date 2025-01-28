package frc.robot.subsystem;

import edu.wpi.first.units.measure.*;
import frc.robot.MOESubsystem;

public class FakeCoralCollector extends MOESubsystem<CoralCollectorInputsAutoLogged> implements CoralCollectorControl {

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
