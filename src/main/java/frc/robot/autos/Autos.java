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
//        autoChooser.addOption("Auto2: S2_E4_CS_C2 ",  ReefToSourceToReef.S2_E4_CS_C(robot));
        autoChooser.addOption("Auto3: S3_G4_PRO", ReefToSource.S3_G4_PRO(robot));
        autoChooser.addOption("Auto4: S4_I1_CS", ReefToSourceL1.S4_I1_CS(robot));
        SmartDashboard.putData("Autos Chooser",autoChooser);

    }
    public static Command getSelectedAuto () {
        return autoChooser.getSelected();
    }
}
