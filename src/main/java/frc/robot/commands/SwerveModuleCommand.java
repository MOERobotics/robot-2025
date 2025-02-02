package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.SwerveDrive;
import frc.robot.subsystem.SwerveDriveControl;

public class SwerveModuleCommand extends Command {
    SwerveDriveControl swerveDrive;
    SendableChooser<Integer> Chooser = new SendableChooser<>();
    Joystick driverJoystick;

    public SwerveModuleCommand(SwerveDriveControl swerveDrive, Joystick driverJoystick){
        this.swerveDrive = swerveDrive;
        this.driverJoystick = driverJoystick;
        addRequirements(swerveDrive);
        Chooser.addOption("FR", 1);
        Chooser.addOption("BL", 2);
        Chooser.addOption("BR", 3);
        Chooser.setDefaultOption("FL", 0);

    }

    @Override
    public void initialize(){
        SmartDashboard.putData("Test Module", Chooser);
    }

    @Override
    public void execute(){
        swerveDrive.driveSingleModule(
                Chooser.getSelected(), -driverJoystick.getRawAxis(1),
                -driverJoystick.getRawAxis(0),
                driverJoystick.getRawAxis(4)
        );

    }

    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return false;
    }

}
