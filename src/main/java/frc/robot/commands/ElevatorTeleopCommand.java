package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.interfaces.ElevatorControl;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class ElevatorTeleopCommand extends Command {
    ElevatorControl elevator;
    Joystick joystick;
    LinearVelocity verticalVelocity = InchesPerSecond.of(0);
    AngularVelocity angularVelocity;
    Distance[] Ls = {Inches.of(25.88), Inches.of(39.72), Inches.of(55.59), Inches.of(71.87), Inches.of(8)};
    private final PIDController pid = new PIDController(0.1, 0.3 ,0);
    Distance targetheight;


    public ElevatorTeleopCommand(ElevatorControl elevator, Joystick joystick) {
        this.elevator = elevator;
        this.joystick = joystick;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        pid.reset();
        pid.setIntegratorRange(-1, 1);
        pid.setIZone(1);
        targetheight = elevator.getHeight();
        verticalVelocity = MetersPerSecond.zero();
        angularVelocity = RadiansPerSecond.zero();
    }

    @Override
    public void execute() {//TODO update magic numbers for velocities
        if(joystick.getPOV(0) == 0){
            targetheight = Ls[0];
        }
        if(joystick.getPOV(0) == 90){
            targetheight = Ls[1];
        }
        if(joystick.getPOV(0) == 180){
            targetheight = Ls[2];
        }
        if(joystick.getPOV(0) == 270){
            targetheight = Ls[3];
        }
        if(joystick.getRawButton(8)){
            targetheight = Ls[4];
        }
        Distance error = elevator.getHeight().minus(targetheight);
        double pidOutput = pid.calculate(error.in(Inches));
        pidOutput = MathUtil.clamp(pidOutput, -1, 1);
        verticalVelocity = InchesPerSecond.of(2).times(pidOutput);
        angularVelocity = DegreesPerSecond.of(2).times(
                MathUtil.applyDeadband(joystick.getRawAxis(0), 0.1)
        );
        elevator.moveVertically(verticalVelocity);
        elevator.moveHorizontally(angularVelocity);
        Logger.recordOutput("verticalspeed", verticalVelocity.in(InchesPerSecond));
        Logger.recordOutput("targetheight", targetheight.in(Inches));
        Logger.recordOutput("pidoutput", pidOutput);
        Logger.recordOutput("heightininches", elevator.getHeight().in(Inches));
    }


    @Override
    public void end(boolean interrupted) {
        elevator.moveHorizontally(RPM.zero());
        elevator.moveVertically(MetersPerSecond.zero());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
