package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.telemetry.PanelsTelemetry;
import dev.nextftc.ftc.ActiveOpMode;

import java.util.LinkedHashMap;
import java.util.Map;

public final class Logger {

    public enum Level {
        INFO,
        CRITICAL
    }

    private static final class LogEntry {
        final Level level;
        final String message;

        LogEntry(Level level, String message) {
            this.level = level;
            this.message = message;
        }
    }

    public static boolean showInfo = true;

    public static void enableInfoLogging() {
        showInfo = true;
    }
    public static void disableInfoLogging() {
        showInfo = false;
    }

    private static final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    private static final Map<String, Map<Integer, LogEntry>> structuredLogs = new LinkedHashMap<>();
    private static int logCounter = 0;

    private Logger() {}

    public static void add(String subsystem, String message) {
        add(subsystem, Level.INFO, message);
    }

    public static void add(String subsystem, Level level, String message) {
        structuredLogs
                .computeIfAbsent(subsystem, k -> new LinkedHashMap<>())
                .put(logCounter++, new LogEntry(level, message));
    }

    public static void log(String subsystem, String name, Object value) {
        add(subsystem, Level.INFO, name + ": " + value);
    }

    public static void panelsLog(String name, Object value) {
        panelsTelemetry.getTelemetry().addData(name, value);
    }

    public static void update() {
        for (Map.Entry<String, Map<Integer, LogEntry>> entry : structuredLogs.entrySet()) {
            String subsystem = entry.getKey();
            Map<Integer, LogEntry> messages = entry.getValue();

            boolean printedHeader = false;

            for (LogEntry log : messages.values()) {
                // LOGIC: Always show CRITICAL. Only show INFO if showInfo is true.
                boolean shouldDisplay = (log.level == Level.CRITICAL) || (log.level == Level.INFO && showInfo);

                if (shouldDisplay) {
                    if (!printedHeader) {
                        ActiveOpMode.telemetry().addLine(
                                "\n" + subsystem.toUpperCase() + " ---------------"
                        );
                        printedHeader = true;
                    }

                    // Add a prefix for Critical logs to make them pop
                    ActiveOpMode.telemetry().addLine(log.message);
                }
            }
        }

        ActiveOpMode.telemetry().update();
        structuredLogs.clear();
        logCounter = 0;
        panelsTelemetry.getTelemetry().update();
    }
}