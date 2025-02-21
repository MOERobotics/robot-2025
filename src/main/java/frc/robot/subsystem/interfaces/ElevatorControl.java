package frc.robot.subsystem.interfaces;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ElevatorControl extends Subsystem {
    @AutoLog
    class ElevatorInputs {
        public Distance extension = Inches.zero();
        public Distance height = Inches.zero();
        public Angle angle = Radians.zero();
        public AngularVelocity horizontalSpeed = RadiansPerSecond.zero();
        public LinearVelocity extensionSpeed = InchesPerSecond.zero();
        public Angle extensionMotorPosition = Rotations.zero();
        public Voltage elevatorVoltage = Volts.zero();
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
