package frc.robot.subsystem;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

public class FakeCoralCollector implements CoralCollectorIO  {
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
    public AngularVelocity getLeftPower() {
        return null;

    }

    @Override
    public AngularVelocity getRightPower() {
        return null;
    }
}
