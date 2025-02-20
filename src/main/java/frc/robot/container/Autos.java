package frc.robot.container;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PathPlannerAutoBuilder;
import frc.robot.autos.start4_place_coral_station;

public class Autos {
    public SendableChooser<Command> autoChooser = new SendableChooser<>();
    public Autos (RobotContainer robot){
        PathPlannerAutoBuilder.configure(robot.getSwerveDrive());
        autoChooser.setDefaultOption("Auto1: Start1 -> AB RED", new PathPlannerAuto("TestAuto1 RED"));
        autoChooser.addOption("Auto2: Test Forward-Right", new PathPlannerAuto("TestAuto Forward-Right"));
        autoChooser.addOption("Auto3: Test Forward-Left", new PathPlannerAuto("TestAuto Forward-Left"));
        autoChooser.addOption("Auto4: Test Arc-Right", new PathPlannerAuto("TestAuto Arc-Right"));
        autoChooser.addOption("Auto4: Test Arc-Left", new PathPlannerAuto("TestAuto Arc-Left"));
        autoChooser.addOption("Auto5: Test Start4 -> IJ -> Coral Station", start4_place_coral_station.buildS4IL4Command(robot));
        SmartDashboard.putData("Autos Chooser",autoChooser);
    }
    public Command getSelectedAuto () {
        return autoChooser.getSelected();
    }
}
