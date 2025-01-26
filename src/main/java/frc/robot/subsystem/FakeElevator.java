package frc.robot.subsystem;

import edu.wpi.first.units.measure.*;
import frc.robot.MOESubsystem;

public class FakeElevator extends MOESubsystem<ElevatorInputsAutoLogged> implements ElevatorSubsystem {

    public FakeElevator(){
        this.setSensors(new ElevatorInputsAutoLogged());
    }

    @Override
    public void readSensors(ElevatorInputsAutoLogged sensors) {
        sensors.angle = this.getAngle();
        sensors.height = this.getHeight();

    }

    @Override
    public void moveVertically(LinearVelocity power) {

    }

    @Override
    public void moveHorizontally(AngularVelocity power) {

    }

}
