package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.ElevatorControl;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystem.interfaces.ElevatorControl.ElevatorHeight.*;

public class ElevatorTeleopCommand extends Command {
    ElevatorControl elevator;
    Joystick joystick;
    LinearVelocity verticalVelocity = InchesPerSecond.of(0);
    AngularVelocity angularVelocity;
    private final PIDController pid = new PIDController(0.4, 0.15 ,0.05);
    Distance targetHeight;


    public ElevatorTeleopCommand(RobotContainer robot, Joystick joystick) {
        this.elevator = robot.getElevator();
        this.joystick = joystick;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        pid.setIntegratorRange(-0.2, 0.5);
        pid.setIZone(1.5);
        targetHeight = elevator.getHeight();
        verticalVelocity = MetersPerSecond.zero();
        angularVelocity = RadiansPerSecond.zero();
    }

    @Override
    public void execute() {//TODO update magic numbers for velocities
        if(joystick.getPOV(0) == 0){
            targetHeight = LEVEL1.measure;
        }
        if(joystick.getPOV(0) == 90){
            targetHeight = LEVEL2.measure;
        }
        if(joystick.getPOV(0) == 180){
            targetHeight = LEVEL3.measure;
        }
        if(joystick.getPOV(0) == 270){
            targetHeight = LEVEL4.measure;
        }
        if(joystick.getRawButton(8)){
            targetHeight = STOW.measure;
        }
        Distance error = elevator.getHeight().minus(targetHeight);
        Logger.recordOutput("PidError",error);
        double pidOutput = pid.calculate(error.in(Inches));
        pidOutput = MathUtil.clamp(pidOutput, -1, 1);
        if(MathUtil.applyDeadband(joystick.getRawAxis(1),0.5)!=0){
            pidOutput = -joystick.getRawAxis(1);
            targetHeight = elevator.getHeight();
        }
        verticalVelocity = FeetPerSecond.of(0.7).times(pidOutput);
        angularVelocity = DegreesPerSecond.of(0.2).times(
                MathUtil.applyDeadband(joystick.getRawAxis(0), 0.1)
        );
        elevator.moveVertically(verticalVelocity);
        elevator.moveHorizontally(angularVelocity);
        Logger.recordOutput("verticalspeed", verticalVelocity.in(InchesPerSecond));
        Logger.recordOutput("targetheight", targetHeight.in(Inches));
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
