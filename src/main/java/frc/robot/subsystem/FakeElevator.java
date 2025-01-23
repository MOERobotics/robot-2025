package frc.robot.subsystem;

import edu.wpi.first.units.measure.*;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

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

    @Override
    public Distance getHeight() {
        return Inches.of(0);
    }

    @Override
    public Angle getAngle() {
        return Degrees.of(0);
    }
}
