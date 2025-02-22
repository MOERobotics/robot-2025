package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.interfaces.CoralHeadControl;
import frc.robot.subsystem.interfaces.ElevatorControl;

import static edu.wpi.first.units.Units.*;

public class CoralHeadAutoCommand extends Command {
    private ElevatorControl elevator;
    CoralHeadControl coralcollector;
    boolean scoring;
    boolean hasCoral;
    boolean stopCoral;
    double ct;

    public CoralHeadAutoCommand(CoralHeadControl coralcollector, boolean scoring, ElevatorControl elevator){
        this.coralcollector = coralcollector;
        this.scoring = scoring;
        this.elevator = elevator;
        addRequirements(coralcollector);
    }


    @Override
    public void initialize() {
        ct = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if(coralcollector.hasCoral()) {
            hasCoral = true;
        }
        if(!coralcollector.hasCoral() && hasCoral){
            stopCoral = true;
        }
        AngularVelocity coralWheelRVelocity, coralWheelLVelocity;
        //eject the coral
        if (scoring){
            hasCoral = false;
            stopCoral = false;
            // when at L1 we need to eject differently because the coral must be sideways
            if (elevator.getHeight().lt(Inches.of(45))){
                coralWheelRVelocity = RPM.of(1.00);
                coralWheelLVelocity = RPM.of(0.30);
            } else {
                coralWheelRVelocity = RPM.of(0.80);
                coralWheelLVelocity = RPM.of(0.80);
            }
        }
        //intake the coral
        if(!scoring && !stopCoral) {
            coralWheelLVelocity = RPM.of(0.20);
            coralWheelRVelocity = RPM.of(0.20);
        }
        else {
            coralWheelRVelocity = RPM.zero();
            coralWheelLVelocity = RPM.zero();
        }


        coralcollector.setCoralVelocity(coralWheelRVelocity, coralWheelLVelocity);

        }

    @Override
    public boolean isFinished() {
        if(!scoring && stopCoral) return true;
       // if(scoring && ct < )
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        coralcollector.setCoralVelocity(RPM.of(0), RPM.of(0));
    }
}
