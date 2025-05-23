package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.AlgaeCollectorControl;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class AlgaeCollectorTeleopCommand extends Command {
    AlgaeCollectorControl algaeCollectorControl;
    Joystick joystick;
    AngularVelocity algaeArmVelocity, algaeWheelVelocity;

    public AlgaeCollectorTeleopCommand(RobotContainer robot, Joystick joystick) {
        this.algaeCollectorControl = robot.getAlgaeCollector();
        this.joystick = joystick;
        addRequirements(algaeCollectorControl);
    }

    @Override
    public void initialize() {
        algaeArmVelocity = RadiansPerSecond.zero();
        algaeWheelVelocity = RPM.zero();
    }

    @Override
    public void execute() {
        algaeArmVelocity = RPM.of(-0.5*MathUtil.applyDeadband(joystick.getRawAxis(5),0.05));

        if (joystick.getRawButton(5)){
            algaeWheelVelocity = RPM.of(1);
        }else if(joystick.getRawButton(6)){
            algaeWheelVelocity = RPM.of(-1);
        }else {
            algaeWheelVelocity = RPM.zero();
        }

        algaeCollectorControl.setArmVelocity(algaeArmVelocity);
        algaeCollectorControl.setWheelVelocity(algaeWheelVelocity);
        Logger.recordOutput("armvelocity", algaeArmVelocity);
        Logger.recordOutput("wheelVelocity", algaeWheelVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        algaeCollectorControl.setArmVelocity(RadiansPerSecond.zero());
        algaeCollectorControl.setWheelVelocity(RPM.zero());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
