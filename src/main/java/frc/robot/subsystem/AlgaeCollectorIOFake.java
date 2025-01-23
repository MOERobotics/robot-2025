package frc.robot.subsystem;

import edu.wpi.first.units.measure.AngularVelocity;

public class AlgaeCollectorIOFake implements AlgaeCollectorIO{

    @Override
    public void processInputs(AlgaeCollectorInputs inputs) {
        AlgaeCollectorIO.super.processInputs(inputs);
    }

    @Override
    public void setCollectorVelocity(AngularVelocity collectorVelocity) {
        AlgaeCollectorIO.super.setCollectorVelocity(collectorVelocity);
    }

    @Override
    public void setWheelVelocity(AngularVelocity wheelVelocity) {
        AlgaeCollectorIO.super.setWheelVelocity(wheelVelocity);
    }

    @Override
    public boolean inStartPosition() {
        return false;
    }

    @Override
    public boolean inCollectPosition() {
        return false;
    }

    @Override
    public boolean hasAlgae() {
        return false;
    }
}
