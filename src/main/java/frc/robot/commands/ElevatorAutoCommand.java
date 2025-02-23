package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.interfaces.ElevatorControl;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

public class ElevatorAutoCommand extends Command {
    private final ElevatorControl elevator;
    private boolean isHold;
    private LinearVelocity pidSpeed = InchesPerSecond.of(0);
//    private Distance[] Ls = {Inches.of(33), Inches.of(40), Inches.of(55.59), Inches.of(81.2), Inches.of(24)};
    private int L;
    private Distance targetheight = Inches.of(0);
    private LinearVelocity maxExtensionSpeed;
    private PIDController pid = new PIDController(0.2, 0.2, 0);

    public ElevatorAutoCommand(
            ElevatorControl elevator,
            Distance targetheight,
            LinearVelocity maxExtensionSpeed,
            boolean isHold
    ){
        this.maxExtensionSpeed = maxExtensionSpeed;
        this.elevator = elevator;
        this.targetheight = targetheight;
        this.isHold = isHold;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        pid.setIntegratorRange(-0.5, 0.5);
        pid.setIZone(1.5);
    }

    @Override
    public void execute() {
        Distance error = elevator.getHeight().minus(targetheight);
        double pidOutput = pid.calculate(error.in(Inches));
        pidOutput = MathUtil.clamp(pidOutput, -1, 1);
        LinearVelocity verticalVelocity = maxExtensionSpeed.times(pidOutput);
        Logger.recordOutput("targetheight", targetheight.in(Inches));
        Logger.recordOutput("height", elevator.getHeight().in(Inches));
        Logger.recordOutput("speed", pidSpeed.in(InchesPerSecond));
        elevator.moveVertically(verticalVelocity);
    }

    @Override
    public boolean isFinished() {
        if(isHold) return false;
        else{
            if(targetheight == elevator.getHeight()) return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.moveVertically(InchesPerSecond.of(0));
    }
}
