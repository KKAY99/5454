package edu.wpi.first.pathweaver;

import java.io.File;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Locale;
import java.util.logging.FileHandler;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.LogManager;
import java.util.logging.LogRecord;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;
import java.util.logging.StreamHandler;

/**
 * Helper class for setting up the application loggers.
 */

final class Loggers {

  private Loggers() {
    throw new UnsupportedOperationException("This is a utility class");
  }

  /**
   * Sets up loggers to print to stdout (rather than stderr) and log to ~/PathWeaver/pathweaver.time.log
   */

  public static void setupLoggers() throws IOException {
    //Set up the global level logger. This handles IO for all loggers.
    final Logger globalLogger = LogManager.getLogManager().getLogger("");

    // Remove the default handlers that stream to System.err
    for (Handler handler : globalLogger.getHandlers()) {
      globalLogger.removeHandler(handler);
    }

    String directory = System.getProperty("user.home") + "/PathWeaver/logs";
      File folder = new File(directory);
    if (!folder.exists()) {
      folder.mkdir();
    }
    String time = DateTimeFormatter.ofPattern("YYYY-MM-dd-HH.mm.ss", Locale.getDefault()).format(LocalDateTime.now());
    final Handler fileHandler = new FileHandler(directory + "/pathweaver." + time + ".log");

    fileHandler.setLevel(Level.INFO);    // Only log INFO and above to disk
    globalLogger.setLevel(Level.CONFIG); // Log CONFIG and higher

    // We need to stream to System.out instead of System.err
    final StreamHandler sh = new StreamHandler(System.out, new SimpleFormatter()) {
      @Override
      public synchronized void publish(final LogRecord record) {
        super.publish(record);
        // For some reason this doesn't happen automatically.
        // This will ensure we get all of the logs printed to the console immediately instead of at shutdown
        flush();
      }
    };
    sh.setLevel(Level.CONFIG); // Log CONFIG and higher to stdout

    globalLogger.addHandler(sh);
    globalLogger.addHandler(fileHandler);
    fileHandler.setFormatter(new SimpleFormatter()); //log in text, not xml

    globalLogger.config("Configuration done."); //Log that we are done setting up the logger

  }

}
