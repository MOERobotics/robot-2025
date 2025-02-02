package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.SwerveDrive;

public class SwerveModuleCommand extends Command {
    SwerveDrive swerveDrive;
    SendableChooser<Integer> Chooser = new SendableChooser<>();

    public SwerveModuleCommand(SwerveDrive swerveDrive){
        this.swerveDrive =swerveDrive;
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

    }

    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return false;
    }

}
