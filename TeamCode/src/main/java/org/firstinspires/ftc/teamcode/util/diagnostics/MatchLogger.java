package org.firstinspires.ftc.teamcode.util.diagnostics;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.Locale;

/**
 * Writes one CSV row per loop to {@code /sdcard/FIRST/data/{tag}_{timestamp}.csv}.
 *
 * <p>Pull the file off the robot after a match and plot it. Most "the robot did something weird"
 * questions are answerable from the log and almost none are answerable from memory: telemetry
 * scrolls past at 50 Hz and nobody is watching it during a match anyway.
 *
 * <h2>The logger knows nothing about the robot</h2>
 * The header is given to the constructor and the cells to {@link #logRow(Object...)}, in the same
 * order. {@code Robot} is the one place that knows every subsystem, so it owns the row; this class
 * owns the file. That keeps {@code util} free of upward dependencies and lets a JVM test write to a
 * temporary directory through {@link #MatchLogger(File, String, String...)}.
 *
 * <h2>The BIOBUZZ schema</h2>
 * {@link #BIOBUZZ_COLUMNS} is the column list {@code Robot} supplies. Three groups earn their
 * place for specific reasons. <b>Target and error</b> ({@code path_completion},
 * {@code trans_error}, {@code heading_error}, {@code heading_hold_deg}, {@code shooter_target_rpm})
 * separate "the robot was here" from "the robot was doing what it was told". <b>Identifiers</b>
 * ({@code macro}, {@code macro_outcome}, {@code intake_mode}, {@code transfer_state}) record what
 * the robot was <em>trying</em> to do. <b>{@code battery_v} with
 * {@code loop_ms}</b> explain most "it behaved differently that time" reports. Follower-derived
 * cells are written as {@code NaN} when there is no follower, so a run on a robot whose drivetrain
 * failed to build is visibly missing data instead of looking like a perfect zero-error run.
 *
 * <h2>Two things it deliberately does</h2>
 * <ul>
 *   <li><b>Catches {@link Exception}, not just {@link IOException}.</b> A null pose or a hardware
 *       read fault would otherwise propagate out of {@code loop()} and end the match over a log
 *       line. Logging must never be able to stop the robot.</li>
 *   <li><b>Flushes every {@link #FLUSH_EVERY_ROWS} rows.</b> A {@code BufferedWriter} only reaches
 *       disk when its 8 KB buffer fills or it is closed, so an abnormal exit would lose the tail
 *       of the match, which is the interesting part.</li>
 * </ul>
 *
 * <p>{@code Locale.US} is pinned when formatting: on a device set to a comma-decimal locale the
 * numbers would otherwise be written with commas and silently corrupt the CSV.
 */
public class MatchLogger {
    private static final String DIR = "FIRST/data";

    /** Rows between disk flushes. Without this the tail of the match is lost on a crash. */
    public static final int FLUSH_EVERY_ROWS = 50;
    /**
     * Logs kept in the directory. Opening a new one deletes the oldest beyond this, so a season of
     * practice does not leave thousands of files on the Control Hub (fixthese D8).
     */
    public static int KEEP_NEWEST = 40;

    /**
     * The column set {@code Robot} supplies for BIOBUZZ, in order. Shared with any analysis script
     * so the two cannot drift apart.
     */
    public static final String[] BIOBUZZ_COLUMNS = {
            "t_ms", "phase", "remaining_s", "loop_ms", "battery_v",
            "pose_x", "pose_y", "pose_h",
            "path_mode", "path_completion", "trans_error", "heading_error",
            "intake_mode", "intake_v", "intake_amps",
            "storage_count", "transfer_state",
            "heading_hold_deg", "aim_lock",
            "shooter_rpm", "shooter_target_rpm",
            "ll_target", "ll_tx", "ll_ty", "localization",
            "macro", "macro_outcome"
    };

    private final BufferedWriter writer;
    private final long startNanos;
    private final File file;
    private final int columns;
    private boolean closed = false;
    private int rowsSinceFlush = 0;
    private int rows = 0;

    /**
     * Opens a log under {@code /sdcard/FIRST/data} on the Robot Controller.
     *
     * @param tag    file name prefix, e.g. {@code "teleop"} or {@code "auto"}
     * @param header column names, one per cell later passed to {@link #logRow(Object...)}
     */
    public MatchLogger(String tag, String... header) throws IOException {
        this(new File(AppUtil.ROOT_FOLDER, DIR), tag, header);
    }

    /**
     * Opens a log in an explicit directory. Production code uses the other constructor; this one
     * exists so a unit test can write to a temporary directory without Android.
     */
    public MatchLogger(File dir, String tag, String... header) throws IOException {
        if (!dir.exists() && !dir.mkdirs()) {
            throw new IOException("could not create " + dir);
        }
        pruneOldest(dir, Math.max(0, KEEP_NEWEST - 1));     // leave room for this one
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        file = new File(dir, tag + "_" + stamp + ".csv");
        writer = new BufferedWriter(new FileWriter(file));
        columns = header.length;
        writeRow((Object[]) header);
        startNanos = System.nanoTime();
    }

    public File getFile() {
        return file;
    }

    /** Deletes the oldest {@code .csv} files in {@code dir} until {@code keep} remain. */
    static void pruneOldest(File dir, int keep) {
        File[] logs = dir.listFiles((d, name) -> name.endsWith(".csv"));
        if (logs == null || logs.length <= keep) return;
        Arrays.sort(logs, (a, b) -> {
            int byTime = Long.compare(a.lastModified(), b.lastModified());
            return byTime != 0 ? byTime : a.getName().compareTo(b.getName());
        });
        for (int i = 0; i < logs.length - keep; i++) {
            //noinspection ResultOfMethodCallIgnored
            logs[i].delete();
        }
    }

    /** Milliseconds since the log was opened, for the {@code t_ms} column. */
    public long elapsedMs() {
        return (System.nanoTime() - startNanos) / 1_000_000L;
    }

    /** Rows written so far, excluding the header. */
    public int getRowCount() {
        return rows;
    }

    /**
     * Logs one row. Cells are written in order; {@code Double}/{@code Float} as {@code %.4f}
     * ({@code NaN} stays {@code NaN}), everything else via {@code toString()} with commas replaced
     * by semicolons so one stray character cannot shift every later column.
     *
     * <p>A row whose length differs from the header is still written, because losing the data is
     * worse than a ragged file, but the mismatch is the caller's bug.
     */
    public void logRow(Object... cells) {
        if (closed) return;
        try {
            writeRow(cells);
            rows++;
            if (++rowsSinceFlush >= FLUSH_EVERY_ROWS) {
                writer.flush();
                rowsSinceFlush = 0;
            }
        } catch (Exception ignored) {
            // Deliberately catches Exception, not just IOException: a null cell's toString or a
            // hardware read fault upstream would otherwise propagate out of loop() and end the
            // match over a log line.
        }
    }

    /** Number of columns declared by the header. */
    public int getColumnCount() {
        return columns;
    }

    public void close() {
        if (closed) return;
        closed = true;
        try {
            writer.flush();
            writer.close();
        } catch (IOException ignored) {
        }
    }

    private void writeRow(Object... cells) throws IOException {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < cells.length; i++) {
            if (i > 0) sb.append(',');
            sb.append(formatCell(cells[i]));
        }
        sb.append('\n');
        writer.write(sb.toString());
    }

    static String formatCell(Object o) {
        if (o instanceof Double || o instanceof Float) {
            return String.format(Locale.US, "%.4f", ((Number) o).doubleValue());
        }
        return String.valueOf(o).replace(',', ';');
    }
}
