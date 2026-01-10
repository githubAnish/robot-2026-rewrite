package org.frogforce503.lib.logging;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.List;
import java.util.function.LongSupplier;

import org.littletonrobotics.junction.Logger;

public class LoggedJVM {
  private List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
  private final String ntroot;

  private class LoggedFunction {
    private final String name;
    private final LongSupplier fcn;

    public LoggedFunction(String name, LongSupplier fcn) {
      this.name = ntroot + "/" + name;
      this.fcn = fcn;
    }

    public void run() {
      Logger.recordOutput(name, fcn.getAsLong());
    }
  }

  private List<LoggedFunction> loggedFunctions = new ArrayList<>();

  public LoggedJVM() {
    this.ntroot = "LoggedJVM";
    Runtime runtime = Runtime.getRuntime();
    System.out.println("JVM Runtime Version: " + Runtime.version().toString());

    if (runtime != null) {
      System.out.println("Available Processors: " + runtime.availableProcessors());
      loggedFunctions.add(new LoggedFunction("Free Memory", runtime::freeMemory));
      loggedFunctions.add(new LoggedFunction("Max Memory", runtime::maxMemory));
      loggedFunctions.add(new LoggedFunction("Total Memory", runtime::totalMemory));
      for (var bean : gcBeans) {
        loggedFunctions.add(
            new LoggedFunction("GC: " + bean.getName() + " Count", bean::getCollectionCount));
        loggedFunctions.add(
            new LoggedFunction("GC: " + bean.getName() + " Time", bean::getCollectionTime));
      }
    } else {
      System.out.println("LoggedJVM Runtime is NULL");
    }
  }

  public void update() {
    for (var f : loggedFunctions) {
      f.run();
    }
  }
}