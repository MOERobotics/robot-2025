package frc.robot.commands.junk;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.interfaces.AlgaeCollectorControl;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class AlgaeArmSpeedMeasureCommand extends Command {

    long startTime;
    Angle startingAngle;
    AlgaeCollectorControl algaeCollector;

    public AlgaeArmSpeedMeasureCommand(AlgaeCollectorControl algaeCollector){
        this.algaeCollector = algaeCollector;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        startingAngle = algaeCollector.getArmAngle();
    }

    @Override
    public void execute() {
        algaeCollector.setArmVelocity(RPM.of(Double.MAX_VALUE));
    }

    @Override
    public void end(boolean interrupted){
        algaeCollector.setArmVelocity(RPM.zero());
        Logger.recordOutput("Algae Collector Angle Travelled", algaeCollector.getArmAngle().minus(startingAngle));
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() > (startTime + 1000);
    }
}
