package frc.robot.subsystem;

import edu.wpi.first.units.measure.AngularVelocity;

public class AlgaeCollectorControlFake implements AlgaeCollectorControl {

    @Override
    public void processInputs(AlgaeCollectorInputs inputs) {
        AlgaeCollectorControl.super.processInputs(inputs);
    }

    @Override
    public void setCollectorVelocity(AngularVelocity collectorVelocity) {
        AlgaeCollectorControl.super.setCollectorVelocity(collectorVelocity);
    }

    @Override
    public void setWheelVelocity(AngularVelocity wheelVelocity) {
        AlgaeCollectorControl.super.setWheelVelocity(wheelVelocity);
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
