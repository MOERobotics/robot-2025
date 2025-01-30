package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ElevatorControl;

import static edu.wpi.first.units.Units.*;

public class ElevatorTeleopCommand extends Command {
    ElevatorControl elevatorControl;
    Joystick joystick;
    LinearVelocity verticalVelocity;
    AngularVelocity angularVelocity;

    public ElevatorTeleopCommand(ElevatorControl elevatorControl, Joystick joystick) {
        this.elevatorControl = elevatorControl;
        this.joystick = joystick;
    }

    @Override
    public void initialize() {
        verticalVelocity = MetersPerSecond.zero();
        angularVelocity = RadiansPerSecond.zero();
    }

    @Override
    public void execute() {//TODO update magic numbers for velocities
        if (joystick.getRawButton(5)) {
            verticalVelocity = MetersPerSecond.of(1);
        } else if (joystick.getRawButton(6)) {
            verticalVelocity = MetersPerSecond.of(-1);
        } else {
            verticalVelocity = MetersPerSecond.zero();
        }

        if (joystick.getRawButton(7)) {
            angularVelocity = RadiansPerSecond.of(1);
        } else if (joystick.getRawButton(8)) {
            angularVelocity = RadiansPerSecond.of(-1);
        } else {
            angularVelocity = RadiansPerSecond.zero();
        }
        elevatorControl.moveVertically(verticalVelocity);
        elevatorControl.moveHorizontally(angularVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorControl.moveHorizontally(RPM.zero());
        elevatorControl.moveVertically(MetersPerSecond.zero());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
