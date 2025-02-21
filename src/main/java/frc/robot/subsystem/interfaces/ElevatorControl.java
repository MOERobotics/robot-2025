package frc.robot.subsystem.interfaces;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystem.interfaces.ElevatorInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ElevatorControl extends Subsystem {
    @AutoLog
    class ElevatorInputs{
        public Distance extension  = Inches.of(0);;
        public Distance height = Inches.of(0);
        public Angle angle  = Radians.of(0);;
        public AngularVelocity horizontalSpeed  = RadiansPerSecond.of(0);;
        public LinearVelocity extensionSpeed = InchesPerSecond.of(0);;

        public Angle extensionMotorPosition;

        public double elevatorVoltage;
        public String elevatorVoltageFromADC;

        public boolean canGoDown;
    }

    public ElevatorInputsAutoLogged getSensors();

    public void moveVertically(LinearVelocity speed);

    public void moveHorizontally(AngularVelocity speed);

    default Distance getExtension() {
        return this.getSensors().extension;
    }
    default boolean canGoDown() {
        return this.getSensors().canGoDown;
    }

    public Distance getPivotHeight();

    default Angle getAngle() {
        return this.getSensors().angle;
    }

    default Distance getHeight() {
        return this.getExtension().times(Math.cos(this.getAngle().in(Radians))).plus(this.getPivotHeight());


    }
}
