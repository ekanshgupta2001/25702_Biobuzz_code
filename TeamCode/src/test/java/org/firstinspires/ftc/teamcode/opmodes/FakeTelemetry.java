package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * A {@link Telemetry} that records every caption and line it is given, so a test can assert what
 * the drivers would have seen. {@code Telemetry} is an interface, so this loads nothing from the
 * Android side of the SDK. Lines accumulate until {@link #clear()}.
 */
public final class FakeTelemetry implements Telemetry {
    public final List<String> lines = new ArrayList<>();
    private int intervalMs = 250;

    public boolean contains(String fragment) {
        for (String line : lines) if (line.contains(fragment)) return true;
        return false;
    }

    public String joined() {
        return String.join("\n", lines);
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        lines.add(caption + ": " + String.format(Locale.US, format, args));
        return null;
    }

    @Override
    public Item addData(String caption, Object value) {
        lines.add(caption + ": " + value);
        return null;
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        lines.add(caption + ": " + valueProducer.value());
        return null;
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        lines.add(caption + ": " + String.format(Locale.US, format, valueProducer.value()));
        return null;
    }

    @Override public boolean removeItem(Item item) { return false; }
    @Override public void clear() { lines.clear(); }
    @Override public void clearAll() { lines.clear(); }
    @Override public Object addAction(Runnable action) { return null; }
    @Override public boolean removeAction(Object token) { return false; }
    @Override public void speak(String text) { }
    @Override public void speak(String text, String languageCode, String countryCode) { }
    @Override public boolean update() { return true; }
    @Override public Line addLine() { lines.add(""); return null; }
    @Override public Line addLine(String lineCaption) { lines.add(lineCaption); return null; }
    @Override public boolean removeLine(Line line) { return false; }
    @Override public boolean isAutoClear() { return true; }
    @Override public void setAutoClear(boolean autoClear) { }
    @Override public int getMsTransmissionInterval() { return intervalMs; }
    @Override public void setMsTransmissionInterval(int ms) { intervalMs = ms; }
    @Override public String getItemSeparator() { return " | "; }
    @Override public void setItemSeparator(String itemSeparator) { }
    @Override public String getCaptionValueSeparator() { return " : "; }
    @Override public void setCaptionValueSeparator(String captionValueSeparator) { }
    @Override public void setDisplayFormat(DisplayFormat displayFormat) { }
    @Override public Log log() { return null; }
}
