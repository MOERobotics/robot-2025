package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.AlgaeCollectorControl;
import frc.robot.subsystem.CoralCollectorControl;

import static edu.wpi.first.units.Units.*;

public class CoralCollectorTeleopCommand extends Command {
    CoralCollectorControl coralCollectorRControl;
    CoralCollectorControl coralcollectorLControl;

    Joystick joystick;
    AngularVelocity coralWheelRVelocity, coralWheelLVelocity;

    public CoralCollectorTeleopCommand(CoralCollectorControl coralCollectorControl, Joystick joystick) {
        this.coralCollectorRControl = coralCollectorRControl;
        this.coralcollectorLControl = coralcollectorLControl;
        this.joystick = joystick;
        addRequirements(algaeCollectorControl);
    }

    @Override
    public void initialize() {
        coralWheelRVelocity = RadiansPerSecond.zero();
        coralWheelLVelocity = RadiansPerSecond.zero();
    }

    @Override
    public void execute() {
        if (joystick.getRawButton(idk)) {
            coralWheelRVelocity = RadiansPerSecond.of(1);
            coralWheelRVelocity = RadiansPerSecond.of(1);

        } else if (joystick.getRawButton(idk)) {
            coralWheelRVelocity = RadiansPerSecond.of(-1);
            coralWheelLVelocity = RadiansPerSecond.of(-1);
        } else {
            coralWheelRVelocity = RadiansPerSecond.zero();
            coralWheelLVelocity = RadiansPerSecond.zero();

        }

        if (joystick.getRawButton(idk)) {
            coralWheelRVelocity = RPM.of(-1);
            coralWheelLVelocity = RPM.of(1);

        } else if (joystick.getRawButton(3)) {
            coralWheelRVelocity = RPM.of(1);
            coralWheelLVelocity = RPM.of(-1);
        } else {
            coralWheelRVelocity = RPM.zero();
            coralWheelLVelocity = RPM.zero();

        }


    }

