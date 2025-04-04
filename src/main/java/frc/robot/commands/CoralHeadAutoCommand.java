package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.interfaces.CoralHeadControl;

import static edu.wpi.first.units.Units.*;

public class CoralHeadAutoCommand extends Command {
    CoralHeadControl coralCollector;
    boolean scoring;
    AngularVelocity rotateSpeed = RPM.of(0);
    Timer timer;

    public CoralHeadAutoCommand(CoralHeadControl coralCollector, boolean scoring, AngularVelocity rotateSpeed){
        this.coralCollector = coralCollector;
        this.scoring = scoring;
        this.rotateSpeed = rotateSpeed;
        this.timer = new Timer();
        addRequirements(coralCollector);
    }
    
    @Override
    public void initialize() {
        coralCollector.setCoralVelocity(RPM.zero(),RPM.zero());
        timer.reset();
    }

    @Override
    public void execute() {
        if (!coralCollector.frontBeam()&&!coralCollector.backBeam()){
            timer.start();
        }
        coralCollector.setCoralVelocity(rotateSpeed, rotateSpeed);
    }

    @Override
    public boolean isFinished() {
        if(!scoring)  return coralCollector.frontBeam()&&coralCollector.backBeam();
        if(scoring) return !coralCollector.frontBeam()&&!coralCollector.backBeam()&&timer.hasElapsed(0.050);
        return false;
    }


    @Override
    public void end(boolean interrupted) {
        coralCollector.setCoralVelocity(RPM.of(0), RPM.of(0));
    }
}
