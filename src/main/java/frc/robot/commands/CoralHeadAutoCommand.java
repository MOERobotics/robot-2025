package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.CoralCollectorControl;

import static edu.wpi.first.units.Units.*;

public class CoralHeadAutoCommand extends Command {
    CoralCollectorControl coralcollector;
    boolean scoring;
    AngularVelocity rotatespeed = RPM.of(0);

    public CoralHeadAutoCommand(CoralCollectorControl coralcollector, boolean scoring, AngularVelocity rotatespeed){
        this.coralcollector = coralcollector;
        this.scoring = scoring;
        this.rotatespeed = rotatespeed;
        addRequirements(coralcollector);
    }

    @Override
    public void execute() {
        coralcollector.setCoralVelocity(rotatespeed, rotatespeed);
    }

    @Override
    public boolean isFinished() {
        if(!scoring)  return coralcollector.hasCoral();
        else return !coralcollector.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        coralcollector.setCoralVelocity(RPM.of(0), RPM.of(0));
    }
}
