package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.interfaces.SwerveDriveControl;

public class SwerveControllerCommand extends Command {
    SwerveDriveControl swerveDrive;

    Joystick joystick;
    public SwerveControllerCommand(SwerveDriveControl swerveDrive, Joystick joystick){
        this.joystick = joystick;
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        double driveX =  -joystick.getRawAxis(1);
        double driveY = -joystick.getRawAxis(0);
        double driveTheta = -joystick.getRawAxis(2);
        driveTheta = MathUtil.applyDeadband(driveTheta, 0.2, 1);
        driveX = MathUtil.applyDeadband(driveX, 0.1, 1) * (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? 1 : -1);
        driveY = MathUtil.applyDeadband(driveY, 0.1, 1) * (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? 1 : -1);
        double speedMultiplier = joystick.getRawButton(7) ? 0.8 : 1;
        swerveDrive.drive(
                2.75*driveX*speedMultiplier,
                2.75*driveY*speedMultiplier,
                1.25*Math.PI*driveTheta*speedMultiplier,
                joystick.getRawButton(8)
        );

    }

    @Override
    public void end(boolean interrupted){
        swerveDrive.drive(0,0,0);
    }
    @Override
    public boolean isFinished(){
        return false;
    }

}
