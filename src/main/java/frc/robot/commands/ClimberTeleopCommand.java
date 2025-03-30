package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.ClimberControl;
import frc.robot.subsystem.interfaces.ElevatorControl;
import frc.robot.subsystem.interfaces.SwerveDriveControl;

import static edu.wpi.first.units.Units.*;

public class ClimberTeleopCommand extends Command {

    ClimberControl climberRear;
    ClimberControl climberMid;

    ElevatorControl elevatorControl;
    SwerveDriveControl swerveDriveControl;
    Long pressTime;



    Joystick joystick;
    public ClimberTeleopCommand(RobotContainer robot, Joystick joystick) {
        this.climberRear = robot.getClimberRear();
        this.climberMid = robot.getClimberMid();
        this.joystick = joystick;
        swerveDriveControl = robot.getSwerveDrive();
        elevatorControl = robot.getElevator();
        this.addRequirements(climberMid, climberRear);
    }

    @Override
    public void initialize() {
        super.initialize();
        pressTime = 0L;
    }

    @Override
    public void execute() {
        double climberPow = 0;
        // Lower Climber arms to climb
        if(joystick.getRawButtonPressed(4)){
            pressTime = System.currentTimeMillis();
        }
        if(joystick.getRawButton(4)){
            climberPow = -Double.MAX_VALUE;
            if(System.currentTimeMillis()-pressTime<2000){
                climberPow=-0.4;
            }
        }
        // Raise climber arms into the air
        if(joystick.getRawButton(3)){
            climberPow = 0.3;
        }

        climberMid.setClimberVelocity(RPM.of(climberPow));
        climberRear.setClimberVelocity(RPM.of(climberPow));

        if(swerveDriveControl.canClimb()){
            elevatorControl.setLEDPattern(LEDPattern.solid(Color.kOrangeRed));
        }

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
