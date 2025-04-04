package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.interfaces.CoralHeadControl;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class CoralHeadAutoCommand extends Command {
    CoralHeadControl coralCollector;
    boolean scoring;
    AngularVelocity rotateSpeed = RPM.of(0);
    Timer stopTimer, startTimer;

    public CoralHeadAutoCommand(CoralHeadControl coralCollector, boolean scoring, AngularVelocity rotateSpeed){
        this.coralCollector = coralCollector;
        this.scoring = scoring;
        this.rotateSpeed = rotateSpeed;
        this.stopTimer = new Timer();
        this.startTimer = new Timer();
        addRequirements(coralCollector);
    }
    
    @Override
    public void initialize() {
        coralCollector.setCoralVelocity(RPM.zero(),RPM.zero());
        startTimer.reset();
        stopTimer.reset();
        startTimer.start();
    }

    @Override
    public void execute() {
        if (!coralCollector.frontBeam()&&!coralCollector.backBeam()){
            stopTimer.start();
        }
        if(!scoring||startTimer.hasElapsed(.050)){
            startTimer.stop();
            coralCollector.setCoralVelocity(rotateSpeed, rotateSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        if(!scoring)  return coralCollector.frontBeam()&&coralCollector.backBeam();
        if(scoring) return !coralCollector.frontBeam()&&!coralCollector.backBeam()&& stopTimer.hasElapsed(0.050);
        return false;
    }


    @Override
    public void end(boolean interrupted) {
        coralCollector.setCoralVelocity(RPM.of(0), RPM.of(0));
    }
}
