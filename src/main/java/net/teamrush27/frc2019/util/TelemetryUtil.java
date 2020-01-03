package net.teamrush27.frc2019.util;

import com.opencsv.CSVWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.stream.Collectors;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class TelemetryUtil {

  private static TelemetryUtil INSTANCE = null;

  public static TelemetryUtil getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new TelemetryUtil();
    }

    return INSTANCE;
  }

  private TelemetryUtil() {}

  private static final Logger LOG = LogManager.getLogger(TelemetryUtil.class);

  private Queue<TelemetryEntry> entries = new ConcurrentLinkedQueue<>();

  public void addEntry(double timestamp, String entryName, String value) {
    entries.offer(new TelemetryEntry(timestamp, entryName, value));
  }

  public void writeToFile(String filePath) throws IOException {
    List<TelemetryEntry> entries = new ArrayList<>();
    entries.addAll(this.entries);
    Collections.sort(entries);

    Set<String> headers = new HashSet<>();
    entries.parallelStream().map(TelemetryEntry::getEntryName).forEach(headers::add);

    Set<Double> timestampSet = new HashSet<>();
    entries.parallelStream().map(TelemetryEntry::getTimestamp).forEach(timestampSet::add);

    List<Double> timestamps = new ArrayList<>();
    timestamps.addAll(timestampSet);
    Collections.sort(timestamps);

    this.entries.clear();
    final File file = new File(filePath);
    if (!file.exists()) {
      file.createNewFile();
    }

    // write headers
    List<String> row = new ArrayList<>();
    row.add("timestamp");
    row.addAll(headers);

    CSVWriter csvWriter = new CSVWriter(new FileWriter(file));
    csvWriter.writeNext(row.toArray(new String[0]));

    timestamps.forEach(
        ts -> {
          row.clear();
          row.add(ts.toString());
          List<TelemetryEntry> filteredEntries =
              entries
                  .parallelStream()
                  .filter(te -> Objects.equals(te.getTimestamp(), ts))
                  .collect(Collectors.toList());

          headers.forEach(
              h -> {
                Optional<TelemetryEntry> ote =
                    filteredEntries
                        .parallelStream()
                        .filter(te -> Objects.equals(te.getEntryName(), h))
                        .findFirst();
                ote.ifPresentOrElse(te -> row.add(te.getValue()), () -> row.add(""));
              });

          csvWriter.writeNext(row.toArray(new String[0]));
        });

    csvWriter.flushQuietly();
    csvWriter.close();

    LOG.info("done writing telemetry");
  }
}
