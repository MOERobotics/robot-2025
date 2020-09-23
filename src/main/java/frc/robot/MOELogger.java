package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class MOELogger {

    static final boolean REPLAY = false;
    public static void setupLogging(LoggedRobot robot) {
        // Set up data logging
        org.littletonrobotics.junction.Logger.recordMetadata("ProjectName", "robot-2025");
        SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
        if (REPLAY) {
            robot.setUseTiming(true);//run as fast as possible
            String logPath = LogFileUtil.findReplayLog();
            WPILOGReader wpilogReader = new WPILOGReader(logPath);
            org.littletonrobotics.junction.Logger.setReplaySource(wpilogReader);
            org.littletonrobotics.junction.Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,"_sim")));
        } else {
            // Write to NetworkTables
            org.littletonrobotics.junction.Logger.addDataReceiver(new NT4Publisher());
            // Save data to file
            org.littletonrobotics.junction.Logger.addDataReceiver(new WPILOGWriter());
        }

        org.littletonrobotics.junction.Logger.start();
    }
}
