package net.teamrush27.frc2019.util;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.util.concurrent.ConcurrentLinkedDeque;

/** Writes data to a CSV file */
public class ReflectingCSVWriter<T> {
  ConcurrentLinkedDeque<T> mObjectsToWrite = new ConcurrentLinkedDeque<>();

  ConcurrentLinkedDeque<String> mLinesToWrite = new ConcurrentLinkedDeque<>();
  PrintWriter mOutput = null;
  Field[] mFields;
  boolean wrote_header = false;

  public ReflectingCSVWriter(String fileName, Class<T> typeClass) {
    mFields = typeClass.getFields();
    try {
      mOutput = new PrintWriter(fileName);
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }
  }

  private void writeHeader(T value) {
    // Write field names.
    StringBuffer line = new StringBuffer();
    for (Field field : mFields) {

      if (line.length() != 0) {
        line.append(", ");
      }
      if (CSVWritable.class.isAssignableFrom(field.getType())) {
        try {
          line.append(((CSVWritable) field.get(value)).header(field.getName()));
        } catch (IllegalAccessException e) {
          e.printStackTrace();
        }
      } else {
        line.append(field.getName());
      }
    }
    mLinesToWrite.add(line.toString());
  }

  public void add(T value) {
    mObjectsToWrite.add(value);
  }

  protected synchronized void writeLine(String line) {
    if (mOutput != null) {
      mOutput.println(line);
    }
  }

  // Call this periodically from any thread to write to disk.
  public void write() {
    T value;

    while ((value = mObjectsToWrite.pollFirst()) != null) {
      if (!wrote_header) {
        writeHeader(value);
        wrote_header = true;
      }

      StringBuffer line = new StringBuffer();
      for (Field field : mFields) {
        if (line.length() != 0) {
          line.append(", ");
        }
        try {
          if (CSVWritable.class.isAssignableFrom(field.getType())) {
            line.append(((CSVWritable) field.get(value)).toCSV());
          } else {
            line.append(field.get(value).toString());
          }
        } catch (IllegalAccessException e) {
          e.printStackTrace();
        }
      }

      mLinesToWrite.add(line.toString());
    }

    String val;
    while ((val = mLinesToWrite.pollFirst()) != null) {
      writeLine(val);
    }
  }

  public synchronized void flush() {
    if (mOutput != null) {
      write();
      mOutput.flush();
    }
  }
}
