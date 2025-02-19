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
    private LinearVelocity maxExtensionSpeed = InchesPerSecond.of(0);
    private PIDController pid = new PIDController(0.1, 0 ,0);
    private boolean isHold;
    private LinearVelocity pidSpeed = InchesPerSecond.of(0);
    private Distance[] Ls = {Inches.of(17.88), Inches.of(31.72), Inches.of(47.59), Inches.of(71.87)};
    private int L;
    private Distance targetheight = Inches.of(0);

    public ElevatorAutoCommand(
            ElevatorControl elevator,
            int L,
            LinearVelocity maxExtensionSpeed,
            boolean isHold
    ){
        this.elevator = elevator;
        this.L = L;
        this.maxExtensionSpeed = maxExtensionSpeed;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        targetheight = Ls[L];
    }

    @Override
    public void execute() {
        Distance error = targetheight.minus(elevator.getHeight());
        double pidOutput = pid.calculate(error.in(Inches));
        pidOutput = MathUtil.clamp(pidOutput, -1, 1);
        LinearVelocity pidSpeed = maxExtensionSpeed.times(-pidOutput);
        elevator.moveVertically(pidSpeed);
        Logger.recordOutput("height", elevator.getHeight().in(Inches));
        Logger.recordOutput("speed", pidSpeed.in(InchesPerSecond));
        Logger.recordOutput("pid", pidOutput);
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
        elevator.moveVertically(InchesPerSecond.of(0));
    }
}
