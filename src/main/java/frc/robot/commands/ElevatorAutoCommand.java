package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ElevatorControl;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

public class ElevatorAutoCommand extends Command {
    private final Distance targetheight;
    private final ElevatorControl elevator;
    private LinearVelocity maxExtensionSpeed = InchesPerSecond.of(0);
    private PIDController pid = new PIDController(0.1, 0 ,0);
    private boolean isHold;
    private LinearVelocity pidSpeed = InchesPerSecond.of(0);

    public ElevatorAutoCommand(
            ElevatorControl elevator,
            Distance targetheight,
            LinearVelocity maxExtensionSpeed,
            boolean isHold
    ){
        this.elevator = elevator;
        this.targetheight = targetheight;
        this.maxExtensionSpeed = maxExtensionSpeed;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Distance error = targetheight.minus(elevator.getHeight());
        double pidOutput = pid.calculate(error.in(Inches));
        pidOutput = MathUtil.clamp(pidOutput, -1, 1);
        LinearVelocity pidSpeed = maxExtensionSpeed.times(pidOutput);
        elevator.moveVertically(pidSpeed);
    }

    @Override
    public boolean isFinished() {
        if(isHold){
            return false;
        }
        if (targetheight.minus(elevator.getHeight()).abs(Inches) < 2) {
            if(Math.abs(pidSpeed.in(InchesPerSecond)) < 2) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.maxExtensionSpeed = InchesPerSecond.of(0);
    }
}
