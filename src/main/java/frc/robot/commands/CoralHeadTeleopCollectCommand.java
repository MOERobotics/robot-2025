package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystem.CoralHeadControl;
import edu.wpi.first.wpilibj.Joystick;

public class CoralHeadTeleopCollectCommand extends Command {
    private final CoralHeadControl coralHead;
    private final Joystick joystick;
    private boolean seenCoral = false;

    private AngularVelocity SPEED = Units.RPM.of(500);

    public CoralHeadTeleopCollectCommand(CoralHeadControl coralHead, Joystick joystick) {
        this.coralHead = coralHead;
        this.joystick = joystick;
        addRequirements(coralHead);
    }

    @Override
    public void initialize() {
        seenCoral = false;
    }

    @Override
    public void execute() {
        if (joystick.getRawButton(2)) {
            coralHead.setCoralVelocity(SPEED, SPEED);
            return;
        }

        if (coralHead.hasCoral()) {
            seenCoral = true;
        }


        if (seenCoral && !coralHead.hasCoral()) {
            coralHead.setCoralVelocity(Units.RPM.zero(), Units.RPM.zero());
        }
    }

    @Override
    public boolean isFinished() {
        return !joystick.getRawButton(2) || (seenCoral && !coralHead.hasCoral());
    }

    @Override
    public void end(boolean interrupted) {
        coralHead.setCoralVelocity(Units.RPM.zero(), Units.RPM.zero());
    }
}
