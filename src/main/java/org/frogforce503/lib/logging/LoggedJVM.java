package org.frogforce503.lib.logging;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.List;
import java.util.function.LongSupplier;

import org.littletonrobotics.junction.Logger;

public final class LoggedJVM {
    private final String rootKey = "LoggedJVM";

    private final List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
    private final List<LoggedFunction> loggedFunctions = new ArrayList<>();

    public LoggedJVM() {
        Runtime runtime = Runtime.getRuntime();

        System.out.println("JVM Runtime Version: " + Runtime.version());
        System.out.println("Available Processors: " + runtime.availableProcessors());

        // Memory stats
        add("Free Memory", runtime::freeMemory);
        add("Max Memory", runtime::maxMemory);
        add("Total Memory", runtime::totalMemory);

        // GC stats
        for (var gc : gcBeans) {
            String base = "GC/" + gc.getName();
            add(base + "/Count", gc::getCollectionCount);
            add(base + "/Time", gc::getCollectionTime);
        }
    }

    public void update() {
        loggedFunctions.forEach(LoggedFunction::log);
    }

    private void add(String name, LongSupplier supplier) {
        loggedFunctions.add(new LoggedFunction(rootKey + "/" + name, supplier));
    }

    private static class LoggedFunction {
        private final String key;
        private final LongSupplier supplier;

        private LoggedFunction(String key, LongSupplier supplier) {
            this.key = key;
            this.supplier = supplier;
        }

        private void log() {
            Logger.recordOutput(key, supplier.getAsLong());
        }
    }
}