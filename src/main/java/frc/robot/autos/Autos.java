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
        autoChooser.addOption("Auto1: S2_E2_CS ",  ReefToSource.S2_E2_CS(robot));
        autoChooser.setDefaultOption("Auto2: S2_E2_CS_C2 ",  ReefToSourceToReef.S2_E2_CS_C2(robot));
        autoChooser.addOption("Auto3: S3_G2_PRO", ReefToSource.S3_G2_PRO(robot));
        SmartDashboard.putData("Autos Chooser",autoChooser);
    }
    public static Command getSelectedAuto () {
        return autoChooser.getSelected();
    }
}
