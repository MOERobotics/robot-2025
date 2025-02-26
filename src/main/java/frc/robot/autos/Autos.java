package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PathPlannerAutoBuilder;
import frc.robot.container.RobotContainer;

public class Autos {
    public static SendableChooser<Command> autoChooser = new SendableChooser<>();
    public static void setupAutos (RobotContainer robot){
        PathPlannerAutoBuilder.configure(robot.getSwerveDrive());
        autoChooser.setDefaultOption("Auto1: Start4 Auto ",  ReefToSource.S2_E3_CS(robot));
        autoChooser.addOption("Auto2: Start 2 Auto ", ReefToSource.S2_E3_CS(robot));
        SmartDashboard.putData("Autos Chooser",autoChooser);
    }
    public static Command getSelectedAuto () {
        return autoChooser.getSelected();
    }
}
