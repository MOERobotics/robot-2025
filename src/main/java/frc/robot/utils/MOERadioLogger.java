package frc.robot.utils;


import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.ProtocolException;
import java.net.URI;
import java.net.URL;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.atomic.AtomicReference;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.NoArgsConstructor;
import lombok.SneakyThrows;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LogTable.LogValue;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static edu.wpi.first.units.Units.*;


public class MOERadioLogger {

    private final Time requestRate = Seconds.of(1);
    private final Time connectTimeout = Milliseconds.of(500);
    private final Time readTimeout = Milliseconds.of(500);

    private final URL statusURL;
    private final Notifier notifier;

    @Builder @NoArgsConstructor @AllArgsConstructor
    public static class RadioState implements LoggableInputs {
        public boolean isInitialized = false;
        public boolean isConnected = false;
        public String statusURL = null;
        public String statusJson = null;
        public String statusError = null;

        @Override
        public void toLog(LogTable table) {
            table.put("IsInitialized", isInitialized);
            table.put("IsConnected", isConnected);
            table.put("StatusJson", statusJson);
            table.put("StatusError", statusError);
            table.put("StatusURL", statusURL);
            table.put("Status", new LogValue(statusJson, "json"));
        }

        @Override
        public void fromLog(LogTable table) {}
    }

    AtomicReference<RadioState> radioState = new AtomicReference<>(new RadioState());

    public MOERadioLogger() { this(getDefaultURL()); }

    public MOERadioLogger(URL statusURL) {
        this.statusURL = statusURL;
        // Launch notifier
        this.notifier = new Notifier(this::onNotify);
        this.notifier.setName("MOERadioLogger");
        this.notifier.startPeriodic(requestRate.in(Seconds));
        this.onNotify();
    }

    public void logRadio() {
        Logger.processInputs("MOERadio", radioState.get());
    }

    //Everything in here runs on a separate thread, do not call Logger stuff
    @SneakyThrows private void onNotify() {
        // Request status from radio
        String response = null;
        InputStream stream = null;
        String error = null;
        try {
            HttpURLConnection connection = (HttpURLConnection) this.statusURL.openConnection();
            connection.setRequestMethod("GET");
            connection.setConnectTimeout((int)connectTimeout.in(Milliseconds));
            connection.setReadTimeout((int)readTimeout.in(Milliseconds));
            stream = connection.getInputStream();
            response = new String(stream.readAllBytes(), StandardCharsets.UTF_8).replaceAll("\\s+", "");
        } catch (IOException e) {
            error = e.toString();
        } finally {
            if (stream != null) stream.close();
        }

        this.radioState.set(RadioState.builder()
            .isInitialized(true)
            .isConnected(response != null)
            .statusJson(response)
            .statusError(error)
            .statusURL(statusURL.toString())
            .build()
        );
    }

    @SneakyThrows private static URL getDefaultURL() {
        int teamNumber = RobotController.getTeamNumber();
        if (teamNumber == 0) teamNumber = 365;
        return new URI(
            String.format(
                "http://10.%d.%d.1/status",
                teamNumber / 100,
                teamNumber % 100
            )
        ).toURL();
    }
}
