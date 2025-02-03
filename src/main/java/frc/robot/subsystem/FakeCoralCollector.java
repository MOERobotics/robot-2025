package frc.robot.subsystem;

import edu.wpi.first.units.measure.*;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.*;

public class FakeCoralCollector extends MOESubsystem<CoralCollectorInputsAutoLogged> implements CoralCollectorControl {

    public FakeCoralCollector() {
        this.setSensors(new CoralCollectorInputsAutoLogged());
        this.getSensors().inFrontReef = false;
        this.getSensors().hasCoral = false;
        this.getSensors().velocityLeft = RPM.zero();
        this.getSensors().velocityRight = RPM.zero();
    }

    @Override
    public void readSensors(CoralCollectorInputsAutoLogged sensors) {
        super.readSensors(sensors);
    }

    @Override
    public void setCoralVelocity(AngularVelocity leftPower, AngularVelocity rightPower) {

    }
}
