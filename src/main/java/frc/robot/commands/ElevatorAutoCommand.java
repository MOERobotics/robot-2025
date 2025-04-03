package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.interfaces.ElevatorControl;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class ElevatorAutoCommand extends Command {
    private final ElevatorControl elevator;
    private final LinearVelocity maxExtensionSpeed;
    private final PIDController pid = new PIDController(0.3, 0.1 ,0.02);
    private final boolean isHold;
    private final Distance targetHeight;



    public ElevatorAutoCommand(
        ElevatorControl elevator,
        Distance targetHeight,
        LinearVelocity maxExtensionSpeed,
        boolean isHold
    ){
        this.elevator = elevator;
        this.targetHeight = targetHeight;
        this.maxExtensionSpeed = maxExtensionSpeed;
        this.isHold = isHold;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        pid.setIntegratorRange(-0.2, 0.3);
        pid.setIZone(1.5);
    }

    @Override
    public void execute() {
        Distance error = elevator.getHeight().minus(targetHeight);
        double pidOutput = pid.calculate(error.in(Inches));
        pidOutput = MathUtil.clamp(pidOutput, -1, 1);
        LinearVelocity pidSpeed = maxExtensionSpeed.times(pidOutput);
        elevator.moveVertically(pidSpeed);
        Logger.recordOutput("verticalspeed", pidSpeed.in(InchesPerSecond));
        Logger.recordOutput("targetheight", targetHeight.in(Inches));
        Logger.recordOutput("pidoutput", pidOutput);
        Logger.recordOutput("heightininches", elevator.getHeight().in(Inches));
        Logger.recordOutput("elevInt", pid.getAccumulatedError()*pid.getI());
    }

    @Override
    public boolean isFinished() {
        if(isHold){
            return false;
        }
        return targetHeight.minus(elevator.getHeight()).abs(Inches) < 1.0 /*&& Math.abs(elevator.getSensors().extensionSpeed.in(InchesPerSecond)) < 30*/;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.moveVertically(InchesPerSecond.of(0));
    }
}
