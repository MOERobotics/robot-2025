package frc.robot.commands.junk;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.interfaces.ElevatorControl;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class ElevatorSpeedMeasureCommand extends Command {

    long startTime;
    Distance startingHeight;
    ElevatorControl elevatorControl;

    public ElevatorSpeedMeasureCommand(ElevatorControl elevatorControl){
        this.elevatorControl = elevatorControl;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        startingHeight = elevatorControl.getExtension();
    }

    @Override
    public void execute() {
        elevatorControl.moveVertically(InchesPerSecond.of(Double.MAX_VALUE));
    }

    @Override
    public void end(boolean interrupted) {
        elevatorControl.moveVertically(InchesPerSecond.of(0));
        Logger.recordOutput("Elevator Distance Travelled", elevatorControl.getExtension().minus(startingHeight));
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() > (startTime + 1000);
    }
}
