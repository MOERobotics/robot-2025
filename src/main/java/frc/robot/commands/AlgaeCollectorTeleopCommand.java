package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.AlgaeCollectorControl;

import static edu.wpi.first.units.Units.*;

public class AlgaeCollectorTeleopCommand extends Command {
    AlgaeCollectorControl algaeCollectorControl;
    Joystick joystick;
    AngularVelocity algaeArmVelocity, algaeWheelVelocity;

    public AlgaeCollectorTeleopCommand(AlgaeCollectorControl algaeCollectorControl, Joystick joystick) {
        this.algaeCollectorControl = algaeCollectorControl;
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
        if (joystick.getRawButton(0)){
            algaeArmVelocity = RadiansPerSecond.of(1);
        }else if(joystick.getRawButton(1)){
            algaeArmVelocity = RadiansPerSecond.of(-1);
        }else {
            algaeArmVelocity = RadiansPerSecond.zero();
        }

        if (joystick.getRawButton(2)){
            algaeWheelVelocity = RPM.of(1);
        }else if(joystick.getRawButton(3)){
            algaeWheelVelocity = RPM.of(-1);
        }else {
            algaeWheelVelocity = RPM.zero();
        }

        algaeCollectorControl.setArmVelocity(algaeArmVelocity);
        algaeCollectorControl.setWheelVelocity(algaeWheelVelocity);
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
