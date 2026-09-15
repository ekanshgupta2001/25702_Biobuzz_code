package org.firstinspires.ftc.teamcode.util.diagnostics;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * Remembers the compiled value of every {@code public static} non-final field of the classes it
 * is given, and reports which ones differ now.
 *
 * <p>The robot's tunables are plain statics so a bench OpMode can nudge them with the dpad. A
 * static lives as long as the Robot Controller app process, so a value bumped on the bench is what
 * the next match OpMode runs with, and nothing used to show that (fixthese R2-A5). The match
 * OpModes call {@link #snapshot} at init and print {@link #changed()} on their cards; the bench
 * OpModes offer {@link #restoreDefaults()} on a button. The snapshot is taken once per process, on
 * the first OpMode's init, which is always before any bench has run its loop.
 */
public final class Tunables {
    private static final Map<Field, Object> defaults = new LinkedHashMap<>();
    private static final Set<Class<?>> snapshotted = new HashSet<>();

    private Tunables() {}

    /** Records the current value of every tunable in {@code classes} not already recorded. */
    public static synchronized void snapshot(Class<?>... classes) {
        for (Class<?> c : classes) {
            if (!snapshotted.add(c)) continue;
            for (Field f : c.getDeclaredFields()) {
                if (!isTunable(f)) continue;
                try {
                    defaults.put(f, f.get(null));
                } catch (IllegalAccessException ignored) {
                    // public static: cannot happen
                }
            }
        }
    }

    /** {@code Class.FIELD = now (default d)} for every recorded tunable whose value differs. */
    public static synchronized List<String> changed() {
        List<String> out = new ArrayList<>();
        for (Map.Entry<Field, Object> e : defaults.entrySet()) {
            Object now = read(e.getKey());
            if (!Objects.equals(now, e.getValue())) {
                out.add(name(e.getKey()) + " = " + now + " (default " + e.getValue() + ")");
            }
        }
        return out;
    }

    /** Puts every recorded tunable back to its compiled value; returns how many changed. */
    public static synchronized int restoreDefaults() {
        int restored = 0;
        for (Map.Entry<Field, Object> e : defaults.entrySet()) {
            if (Objects.equals(read(e.getKey()), e.getValue())) continue;
            try {
                e.getKey().set(null, e.getValue());
                restored++;
            } catch (IllegalAccessException ignored) {
                // public static: cannot happen
            }
        }
        return restored;
    }

    /** How many tunables are on record. */
    public static synchronized int size() {
        return defaults.size();
    }

    /** Forgets the snapshot so a test can take a fresh one. */
    public static synchronized void resetForTests() {
        defaults.clear();
        snapshotted.clear();
    }

    private static boolean isTunable(Field f) {
        int m = f.getModifiers();
        return Modifier.isPublic(m) && Modifier.isStatic(m) && !Modifier.isFinal(m);
    }

    private static Object read(Field f) {
        try {
            return f.get(null);
        } catch (IllegalAccessException e) {
            return null;
        }
    }

    private static String name(Field f) {
        return f.getDeclaringClass().getSimpleName() + "." + f.getName();
    }
}
