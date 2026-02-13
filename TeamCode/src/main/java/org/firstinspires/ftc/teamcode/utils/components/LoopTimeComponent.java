package org.firstinspires.ftc.teamcode.utils.components;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class LoopTimeComponent implements Component {

    public static final LoopTimeComponent INSTANCE = new LoopTimeComponent();

    private long lastTime = -1;

    @Override
    public void preWaitForStart() {
        update();
    }

    @Override
    public void preUpdate() {
        update();
    }

    private void update() {
        // System.nanoTime() is the Java equivalent to Monotonic.markNow()
        long currentTime = System.nanoTime();

        if (lastTime != -1) {
            // Calculate delta in nanoseconds, then convert to milliseconds
            double loopTimeMs = (currentTime - lastTime) / 1_000_000.0;
            ActiveOpMode.telemetry().addData("Loop Time", loopTimeMs + " ms");
        }

        lastTime = currentTime;
    }
}