package frc.robot.subsystem;

import edu.wpi.first.units.measure.AngularVelocity;

public class AlgaeCollectorControlFake implements AlgaeCollectorControl {

    @Override
    public AlgaeCollectorInputsAutoLogged getSensors() {
        return null;
    }

    @Override
    public void setWheelVelocity(AngularVelocity wheelVelocity) {
    }

    @Override
    public void setWheelState(WheelState wheelState) {

    }
}
