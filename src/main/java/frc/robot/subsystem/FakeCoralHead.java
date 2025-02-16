package frc.robot.subsystem;

import edu.wpi.first.units.measure.*;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.*;

public class FakeCoralHead extends MOESubsystem<CoralHeadInputsAutoLogged> implements CoralHeadControl {

    public FakeCoralHead() {
        this.setSensors(new CoralHeadInputsAutoLogged());
        this.getSensors().inFrontReef = false;
        this.getSensors().hasCoral = false;
        this.getSensors().velocityLeft = RPM.zero();
        this.getSensors().velocityRight = RPM.zero();
    }

    @Override
    public void readSensors(CoralHeadInputsAutoLogged sensors) {
        super.readSensors(sensors);
    }

    @Override
    public void setCoralVelocity(AngularVelocity leftAngularVelocity, AngularVelocity rightAngularVelocity) {

    }
}
