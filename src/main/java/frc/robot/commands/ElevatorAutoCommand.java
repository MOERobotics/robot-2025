package frc.robot.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ElevatorControl;

public class ElevatorAutoCommand extends Command {
    private final Distance targetheight;
    private final ElevatorControl elevator;

    public ElevatorAutoCommand(ElevatorControl elevator, Distance targetheight){
        this.elevator = elevator;
        this.targetheight = targetheight;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
        elevator.getHeight();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
