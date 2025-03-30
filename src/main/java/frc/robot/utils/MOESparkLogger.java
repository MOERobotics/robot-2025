package frc.robot.utils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class MOESparkLogger implements LoggableInputs {
    private final SparkMax spark;
    private final AbsoluteEncoder absoluteEncoder;
    private final SparkLimitSwitch forwardLimit, reverseLimit;
    private final RelativeEncoder relativeEncoder;
    private final SparkMaxConfigAccessor config;

    public MOESparkLogger(SparkMax spark) {
        this.spark = spark;
        this.absoluteEncoder = spark.getAbsoluteEncoder();
        this.forwardLimit = spark.getForwardLimitSwitch();
        this.reverseLimit = spark.getReverseLimitSwitch();
        this.relativeEncoder = spark.getEncoder();
        this.config = spark.configAccessor;
    }

    private static String printByteArray(byte[] buffer) {
        StringBuilder out = new StringBuilder(buffer.length * 2);
        for (byte b : buffer) {
            out.append(String.format("%02X", b));
        }
        return out.toString();
    }

    @Override
    public void toLog(LogTable logTable) {

        logTable.put("deviceID", spark.getDeviceId());
        logTable.put("type", spark.getMotorType().name());
        logTable.put("setpoint", spark.get());
        logTable.put("output%", spark.getAppliedOutput());
        logTable.put("outVoltage", spark.getBusVoltage() * spark.getAppliedOutput());
        logTable.put("busVoltage", spark.getBusVoltage());
        logTable.put("temperature", spark.getMotorTemperature());
        logTable.put("current", spark.getOutputCurrent());

        logTable.put("forwardLimit/isPressed", forwardLimit.isPressed());
        logTable.put("forwardLimit/isEnabled", config.limitSwitch.getForwardLimitSwitchEnabled());
        logTable.put("forwardLimit/type", config.limitSwitch.getForwardSwitchType().name());

        logTable.put("reverseLimit/isPressed", reverseLimit.isPressed());
        logTable.put("reverseLimit/isEnabled", config.limitSwitch.getReverseLimitSwitchEnabled());
        logTable.put("reverseLimit/type", config.limitSwitch.getReverseSwitchType().name());

        logTable.put("absoluteEncoder/position", absoluteEncoder.getPosition());
        logTable.put("absoluteEncoder/velocity", absoluteEncoder.getVelocity());
        logTable.put("absoluteEncoder/inverted", config.absoluteEncoder.getInverted());
        logTable.put("absoluteEncoder/offset", config.absoluteEncoder.getZeroOffset());

        logTable.put("relativeEncoder/position", relativeEncoder.getPosition());
        logTable.put("relativeEncoder/velocity", relativeEncoder.getVelocity());
        logTable.put("relativeEncoder/inverted", config.encoder.getInverted());
        logTable.put("relativeEncoder/countPerRev", config.encoder.getCountsPerRevolution());

        logTable.put("config/serial", printByteArray(spark.getSerialNumber()));
        logTable.put("config/firmware", spark.getFirmwareString());
        logTable.put("config/idleMode", config.getIdleMode());
        logTable.put("config/inverted", config.getInverted());
        logTable.put("config/leader", config.getFollowerModeLeaderId());
        logTable.put("config/leaderInverted", config.getFollowerModeInverted());

    }

    @Override
    public void fromLog(LogTable logTable) {

    }
}
