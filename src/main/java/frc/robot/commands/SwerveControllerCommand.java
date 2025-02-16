package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.SwerveDrive;

public class SwerveControllerCommand extends Command {
    SwerveDrive swerveDrive;


    public SwerveControllerCommand(SwerveDrive swerveDrive){
        this.swerveDrive =swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize(){

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
