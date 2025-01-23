package frc.robot.subsystem;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public class FakeElevator implements ElevatorSubsystem {

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
