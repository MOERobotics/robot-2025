package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.interfaces.CoralHeadControl;

import static edu.wpi.first.units.Units.*;

public class CoralHeadAutoCommand extends Command {
    CoralHeadControl coralCollector;
    boolean scoring;
    boolean outCoral;
    AngularVelocity rotateSpeed = RPM.of(0);

    public CoralHeadAutoCommand(CoralHeadControl coralCollector, boolean scoring, AngularVelocity rotateSpeed){
        this.coralCollector = coralCollector;
        this.scoring = scoring;
        this.rotateSpeed = rotateSpeed;

        addRequirements(coralCollector);
    }
    
    @Override
    public void initialize() {
        coralCollector.setCoralVelocity(RPM.zero(),RPM.zero());
        outCoral = false;
    }

    @Override
    public void execute() {
        if (coralCollector.backBeam()){
            outCoral = true;
        }
        coralCollector.setCoralVelocity(rotateSpeed, rotateSpeed);
    }

    @Override
    public boolean isFinished() {
        if(!scoring)  return coralCollector.frontBeam()&&coralCollector.backBeam();
        if(scoring) return outCoral&&!coralCollector.backBeam();
        return false;
    }


    @Override
    public void end(boolean interrupted) {
        coralCollector.setCoralVelocity(RPM.of(0), RPM.of(0));
    }
}
