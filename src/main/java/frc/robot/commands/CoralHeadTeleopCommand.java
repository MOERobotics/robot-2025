package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.CoralCollector;
import frc.robot.subsystem.CoralCollectorControl;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class CoralHeadTeleopCommand extends Command {
    CoralCollectorControl coralCollectorControl;
    Joystick joystick;

    private boolean preparingCoral = false;


    public CoralHeadTeleopCommand(CoralCollectorControl coralCollectorControl, Joystick joystick) {
        this.coralCollectorControl = coralCollectorControl;
        this.joystick = joystick;
    }

    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        AngularVelocity coralWheelRVelocity, coralWheelLVelocity;
        if (preparingCoral && joystick.getRawButtonPressed(4)) preparingCoral = false;
            else if (joystick.getRawButtonPressed(4)) preparingCoral = true;
        Logger.recordOutput("CoralHeadTeleopCommand/preparingCoral", preparingCoral);
        if (preparingCoral) {
            if (coralCollectorControl.hasCoral()) {

                coralWheelRVelocity = RPM.of(0.5);
                coralWheelLVelocity = RPM.of(0.5);
            } else {
                coralWheelRVelocity = RPM.zero();
                coralWheelLVelocity = RPM.zero();
                preparingCoral = false;
            }
        }else {
            if (joystick.getRawButton(1)) {
                coralWheelRVelocity = RPM.of(1);
                coralWheelLVelocity = RPM.of(1);
            } else {
                coralWheelRVelocity = RPM.zero();
                coralWheelLVelocity = RPM.zero();
            }
        }

        coralCollectorControl.setCoralVelocity(coralWheelRVelocity,coralWheelLVelocity);


    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {

        return false;
    }
}

