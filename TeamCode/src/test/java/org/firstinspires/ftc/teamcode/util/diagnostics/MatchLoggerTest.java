package org.firstinspires.ftc.teamcode.util.diagnostics;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.List;

/**
 * Tests for the CSV writer, against a temporary directory so no Android is needed.
 *
 * <p>The formatting rules are the interesting part: a locale-dependent decimal separator or an
 * unescaped comma in a macro name would corrupt every later column without any error.
 */
public class MatchLoggerTest {
    private enum Mode { INTAKING }

    private File dir;

    @Before
    public void setUp() throws IOException {
        dir = Files.createTempDirectory("matchlogger").toFile();
        MatchLogger.KEEP_NEWEST = 40;
    }

    @After
    public void tearDown() {
        File[] files = dir.listFiles();
        if (files != null) for (File f : files) f.delete();
        dir.delete();
    }

    private List<String> lines(MatchLogger logger) throws IOException {
        return Files.readAllLines(logger.getFile().toPath(), StandardCharsets.UTF_8);
    }

    @Test
    public void writesTheHeaderFirst() throws IOException {
        MatchLogger logger = new MatchLogger(dir, "test", "a", "b", "c");
        logger.close();
        List<String> lines = lines(logger);
        assertEquals(1, lines.size());
        assertEquals("a,b,c", lines.get(0));
        assertEquals(3, logger.getColumnCount());
    }

    @Test
    public void openingALogPrunesTheOldestBeyondKeepNewest() throws IOException {
        // fixthese D8: one CSV per OpMode run, forever, is thousands of files by mid-season.
        MatchLogger.KEEP_NEWEST = 3;
        for (int i = 0; i < 5; i++) {
            File old = new File(dir, "teleop_2026010" + i + "_000000.csv");
            Files.write(old.toPath(), ("old " + i).getBytes(StandardCharsets.UTF_8));
            assertTrue(old.setLastModified(1_000_000L * (i + 1)));      // i = 0 is the oldest
        }
        File other = new File(dir, "notes.txt");
        Files.write(other.toPath(), "keep".getBytes(StandardCharsets.UTF_8));

        MatchLogger logger = new MatchLogger(dir, "auto", "a");
        logger.close();

        File[] csvs = dir.listFiles((d, name) -> name.endsWith(".csv"));
        assertEquals("two oldest gone, two kept, plus the new one", 3, csvs.length);
        assertTrue(!new File(dir, "teleop_20260100_000000.csv").exists());
        assertTrue(!new File(dir, "teleop_20260101_000000.csv").exists());
        assertTrue(new File(dir, "teleop_20260104_000000.csv").exists());
        assertTrue("only logs are touched", other.exists());
    }

    @Test
    public void fileNameCarriesTheTag() throws IOException {
        MatchLogger logger = new MatchLogger(dir, "auto", "a");
        logger.close();
        assertTrue(logger.getFile().getName().startsWith("auto_"));
        assertTrue(logger.getFile().getName().endsWith(".csv"));
    }

    @Test
    public void formatsNumbersEnumsAndStrings() throws IOException {
        MatchLogger logger = new MatchLogger(dir, "test", "d", "f", "i", "e", "s", "nan", "null");
        logger.logRow(1.5, 2.25f, 7, Mode.INTAKING, "plain", Double.NaN, null);
        logger.close();
        assertEquals("1.5000,2.2500,7,INTAKING,plain,NaN,null", lines(logger).get(1));
    }

    @Test
    public void commasInsideCellsAreReplacedSoColumnsCannotShift() throws IOException {
        MatchLogger logger = new MatchLogger(dir, "test", "macro", "next");
        logger.logRow("shoot,all", "x");
        logger.close();
        assertEquals("shoot;all,x", lines(logger).get(1));
    }

    @Test
    public void flushesBeforeCloseOnceEnoughRowsAccumulate() throws IOException {
        MatchLogger logger = new MatchLogger(dir, "test", "n");
        for (int i = 0; i < MatchLogger.FLUSH_EVERY_ROWS; i++) logger.logRow(i);
        // Not closed yet: the flush at FLUSH_EVERY_ROWS must already have reached the disk.
        List<String> lines = lines(logger);
        assertEquals(1 + MatchLogger.FLUSH_EVERY_ROWS, lines.size());
        logger.close();
    }

    @Test
    public void countsRowsAndIgnoresWritesAfterClose() throws IOException {
        MatchLogger logger = new MatchLogger(dir, "test", "n");
        logger.logRow(1);
        logger.logRow(2);
        logger.close();
        logger.logRow(3);
        assertEquals(2, logger.getRowCount());
        assertEquals(3, lines(logger).size());
    }

    @Test
    public void biobuzzSchemaHasNoDuplicateColumns() {
        String[] cols = MatchLogger.BIOBUZZ_COLUMNS;
        for (int i = 0; i < cols.length; i++) {
            for (int j = i + 1; j < cols.length; j++) {
                assertTrue("duplicate column " + cols[i], !cols[i].equals(cols[j]));
            }
        }
        assertEquals("t_ms", cols[0]);
    }
}
