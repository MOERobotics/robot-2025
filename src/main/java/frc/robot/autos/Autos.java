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
        autoChooser.setDefaultOption("Auto1: S2_E4_CS ",  ReefToSource.S2_E4_CS(robot));
        autoChooser.addOption("Auto2: S2_F4_CS_C4_CS ",  ReefToSourceToReefToSource.S2_F4_CS_C4_CS(robot));
        autoChooser.addOption("Auto3: S3_G4_PRO", ReefToProcessor.S3_G4_PRO(robot));
        autoChooser.addOption("Auto4: S2_E4_CS_C4_CS_D4", ReefToSourceToReefToSourceToReef.S2_E4_CS_C4_CS_D4(robot));
        autoChooser.addOption("Auto5: S4_I4_CS_K4_CS_L4", ReefToSourceToReefToSourceToReef.S4_I4_CS_K4_CS_L4(robot));
        SmartDashboard.putData("Autos Chooser",autoChooser);

    }
    public static Command getSelectedAuto () {
        return autoChooser.getSelected();
    }
}
