package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystem.interfaces.SwerveDriveControl;

public class PathPlannerAutoBuilder {
    public static void configure(SwerveDriveControl drive) {
        // Assuming this is a method in your drive subsystem

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }

        AutoBuilder.configure(
                drive::getPose,
                drive::resetPose,
                drive::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> drive.drive(speeds),
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                drive
            );
        }
    }

