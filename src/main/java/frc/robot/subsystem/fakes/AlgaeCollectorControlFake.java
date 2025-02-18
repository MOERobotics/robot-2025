package frc.robot.subsystem.fakes;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystem.interfaces.AlgaeCollectorInputsAutoLogged;
import frc.robot.subsystem.interfaces.AlgaeCollectorControl;

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
