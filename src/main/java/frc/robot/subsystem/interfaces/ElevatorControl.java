package frc.robot.subsystem.interfaces;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystem.interfaces.ElevatorInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

public interface ElevatorControl extends Subsystem {
    @AutoLog
    class ElevatorInputs{
        public Distance extension;
        public Distance height;
        public Angle angle;
        public AngularVelocity horizontalSpeed;
        public LinearVelocity extensionSpeed;

        public Angle extensionMotorPosition;

        public double elevatorVoltage;
        public String elevatorVoltageFromADC;

        public boolean canGoDown;
    }

    Distance heightL1 = Inches.of(33);
    Distance heightL2 = Inches.of(40);
    Distance heightL3 = Inches.of(55.59);
    Distance heightL4 = Inches.of(81.2);
    Distance heightChute = Inches.of(24);

    public ElevatorInputsAutoLogged getSensors();

    public void setTargetHeight(Distance targetHeight);

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
