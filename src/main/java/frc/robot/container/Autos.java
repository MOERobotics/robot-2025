package frc.robot.container;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PathPlannerAutoBuilder;
import frc.robot.autos.start2_place_coral_station;

public class Autos {
    public SendableChooser<Command> autoChooser = new SendableChooser<>();
    public Autos (RobotContainer robot){
        PathPlannerAutoBuilder.configure(robot.getSwerveDrive());
//        autoChooser.setDefaultOption("Auto1: Start1 -> AB RED", new PathPlannerAuto("TestAuto1 RED"));
//        autoChooser.addOption("Auto2: Test Forward-Right", new PathPlannerAuto("TestAuto Forward-Right"));
//        autoChooser.addOption("Auto3: Test Forward-Left", new PathPlannerAuto("TestAuto Forward-Left"));
//        autoChooser.addOption("Auto4: Test Arc-Right", new PathPlannerAuto("TestAuto Arc-Right"));
//        autoChooser.addOption("Auto4: Test Arc-Left", new PathPlannerAuto("TestAuto Arc-Left"));
        autoChooser.addOption("Auto5: Test Start4 -> Score -> Coral Station", start2_place_coral_station.buildS2EL4Command(robot));
        SmartDashboard.putData("Autos Chooser",autoChooser);
    }
    public Command getSelectedAuto () {
        return autoChooser.getSelected();
    }
}
