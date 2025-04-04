package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.hal.CANAPIJNI;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.MOERadioLogger;
import lombok.SneakyThrows;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.lang.reflect.Field;
import java.net.URI;
import java.util.Map;

import static edu.wpi.first.units.Units.Microseconds;

public class MOELogger {

    static final boolean REPLAY = false;
    static MOERadioLogger radioLogger;


    private static Robot _robot;
    private static int heartbeat = 0;
    private static int frequency = 0;

    @SneakyThrows
    public static void setupLogging(Robot robot) {
        // Set up data logging
        _robot = robot;
        org.littletonrobotics.junction.Logger.recordMetadata("ProjectName", "robot-2025");
        SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

        LoggedPowerDistribution.getInstance(
            robot.robot.getPdh().getModule(),
            robot.robot.getPdh().getType()
        );
        if (REPLAY) {
            //run as fast as possible
            robot.setUseTiming(true);
            String logPath = LogFileUtil.findReplayLog();
            WPILOGReader wpilogReader = new WPILOGReader(logPath);
            org.littletonrobotics.junction.Logger.setReplaySource(wpilogReader);
            org.littletonrobotics.junction.Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,"_sim")));
        } else {
            org.littletonrobotics.junction.Logger.addDataReceiver(new NT4Publisher());
            org.littletonrobotics.junction.Logger.addDataReceiver(new WPILOGWriter());
        }

        radioLogger = new MOERadioLogger();
        org.littletonrobotics.junction.Logger.start();
        frequency = (int) Math.round(1/robot.getPeriod());

    }

    @SneakyThrows
    public static void log() {
        radioLogger.logRadio();

        Field _watchdog = IterativeRobotBase.class.getDeclaredField("m_watchdog");
        _watchdog.setAccessible(true);
        Watchdog watchdog = (Watchdog) _watchdog.get(_robot);
        Field _tracer = Watchdog.class.getDeclaredField("m_tracer");
        _tracer.setAccessible(true);
        Tracer tracer = (Tracer) _tracer.get(watchdog);
        Field _epochs = Tracer.class.getDeclaredField("m_epochs");
        _epochs.setAccessible(true);
        Map<String, Long> epochs = (Map<String, Long>) _epochs.get(tracer);

        epochs.forEach(
            (type, length) -> Logger.recordOutput("Runtime/"+type, Microseconds.of(length))
        );


        Logger.recordOutput("heartbeat", heartbeat = (heartbeat + 1) % frequency);
    }



}
