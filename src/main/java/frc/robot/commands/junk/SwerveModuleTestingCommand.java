package frc.robot.commands.junk;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.SwerveDrive;
import frc.robot.subsystem.interfaces.SwerveDriveControl;
import frc.robot.subsystem.SwerveModule;

public class SwerveModuleTestingCommand extends Command {
    SwerveDriveControl swerveDrive;
    SendableChooser<Integer> Chooser = new SendableChooser<>();
    Joystick driverJoystick;

    public SwerveModuleTestingCommand(SwerveDriveControl swerveDrive, Joystick driverJoystick){
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
        SwerveModule chosenModule;

        if (!(swerveDrive instanceof  SwerveDrive)) return;
        SwerveDrive _swerveDrive = (SwerveDrive)swerveDrive;

        chosenModule = switch (Chooser.getSelected()) {
            case 1 -> _swerveDrive.swerveModuleFR;
            case 2 -> _swerveDrive.swerveModuleBL;
            case 3 -> _swerveDrive.swerveModuleBR;
            default -> _swerveDrive.swerveModuleFL;
        };
        chosenModule.driveMotor.set(-driverJoystick.getRawAxis(1));
        chosenModule.pivotMotor.set(driverJoystick.getRawAxis(2));
        if (chosenModule != _swerveDrive.swerveModuleBR) {
            _swerveDrive.swerveModuleBR.driveMotor.set(0);
            _swerveDrive.swerveModuleBR.pivotMotor.set(0);
        }
        if (chosenModule != _swerveDrive.swerveModuleFR) {
            _swerveDrive.swerveModuleFR.driveMotor.set(0);
            _swerveDrive.swerveModuleFR.pivotMotor.set(0);
        }
        if (chosenModule != _swerveDrive.swerveModuleBL) {
            _swerveDrive.swerveModuleBL.driveMotor.set(0);
            _swerveDrive.swerveModuleBL.pivotMotor.set(0);
        }
        if (chosenModule != _swerveDrive.swerveModuleFL) {
            _swerveDrive.swerveModuleFL.driveMotor.set(0);
            _swerveDrive.swerveModuleFL.pivotMotor.set(0);
        }

    }

    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return false;
    }

}
