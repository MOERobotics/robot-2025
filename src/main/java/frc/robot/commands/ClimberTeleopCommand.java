package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.interfaces.ClimberControl;

import static edu.wpi.first.units.Units.*;

public class ClimberTeleopCommand extends Command {

    ClimberControl climberRear;
    ClimberControl climberMid;

    Joystick joystick;
    public ClimberTeleopCommand(ClimberControl climberRear, ClimberControl climberMid, Joystick joystick) {
        this.climberRear = climberRear;
        this.climberMid = climberMid;
        this.joystick = joystick;
        this.addRequirements(climberMid, climberRear);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        double climberPow = 0;
        // TODO: button values are temporary
        // Lower Climber arms to climb
        if(joystick.getRawButton(4)){
            climberPow = -Double.MAX_VALUE;
        }
        // Raise climber arms into the air
        if(joystick.getRawButton(3)){
            climberPow = 0.3;
        }


        climberMid.setClimberVelocity(RPM.of(climberPow));
        climberRear.setClimberVelocity(RPM.of(climberPow));

    }

    @Override
    public void end(boolean interrupted) {
        climberMid.setClimberVelocity(RPM.zero());
        climberRear.setClimberVelocity(RPM.zero());

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
