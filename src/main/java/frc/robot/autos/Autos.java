package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PathPlannerAutoBuilder;
import frc.robot.container.RobotContainer;

public class Autos {
    public static SendableChooser<Command> autoChooser = new SendableChooser<>();
    public static void setupAutos (RobotContainer robot){
        PathPlannerAutoBuilder.configure(robot.getSwerveDrive());
        autoChooser.setDefaultOption("Auto1: Start1 -> AB RED", new PathPlannerAuto("TestAuto1 RED"));
        autoChooser.addOption("Auto2: Test Forward-Right", new PathPlannerAuto("TestAuto Forward-Right"));
        autoChooser.addOption("Auto3: Test Forward-Left", new PathPlannerAuto("TestAuto Forward-Left"));
        autoChooser.addOption("Auto4: Test Arc-Right", new PathPlannerAuto("TestAuto Arc-Right"));
        autoChooser.addOption("Auto4: Test Arc-Left", new PathPlannerAuto("TestAuto Arc-Left"));
        SmartDashboard.putData("Autos Chooser",autoChooser);
    }
    public static Command getSelectedAuto () {
        return autoChooser.getSelected();
    }
}
