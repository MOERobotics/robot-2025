package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.interfaces.ElevatorControl;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

public class ElevatorAutoCommand extends Command {
    private final ElevatorControl elevator;
    private final LinearVelocity maxExtensionSpeed;
    private final PIDController pid = new PIDController(0.2, 0.1 ,0);
    private final boolean isHold;
    private final Distance targetheight;



    public ElevatorAutoCommand(
        ElevatorControl elevator,
        Distance targetheight,
        LinearVelocity maxExtensionSpeed,
        boolean isHold
    ){
        this.elevator = elevator;
        this.targetheight = targetheight;
        this.maxExtensionSpeed = maxExtensionSpeed;
        this.isHold = isHold;
        addRequirements(elevator);
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
            if(Math.abs(elevator.getSensors().extensionSpeed.in(InchesPerSecond)) < 2) {
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
