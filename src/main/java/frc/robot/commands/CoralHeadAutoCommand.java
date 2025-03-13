package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.interfaces.CoralHeadControl;

import static edu.wpi.first.units.Units.*;

public class CoralHeadAutoCommand extends Command {
    CoralHeadControl coralCollector;
    boolean scoring;
    boolean hasCoral;
    AngularVelocity rotatespeed = RPM.of(0);

    public CoralHeadAutoCommand(CoralHeadControl coralcollector, boolean scoring, AngularVelocity rotatespeed){
        this.coralCollector = coralcollector;
        this.scoring = scoring;
        this.rotatespeed = rotatespeed;

        addRequirements(coralcollector);
    }

    @Override
    public void execute() {
        if (coralCollector.hasCoral()) {
            hasCoral = true;
        }
        coralCollector.setCoralVelocity(rotatespeed, rotatespeed);
    }

    @Override
    public boolean isFinished() {
        if(!scoring)  return hasCoral&&!coralCollector.hasCoral();
        else return false;
    }

    @Override
    public void end(boolean interrupted) {
        coralCollector.setCoralVelocity(RPM.of(0), RPM.of(0));
    }
}
