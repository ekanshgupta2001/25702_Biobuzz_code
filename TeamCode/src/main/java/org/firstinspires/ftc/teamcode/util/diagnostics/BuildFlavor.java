package org.firstinspires.ftc.teamcode.util.diagnostics;

/**
 * Which APK is installed. The AutoTune library is on the classpath only in a {@code -Ptuning}
 * build (see {@code build.dependencies.gradle}), and it opens a web server the moment it loads,
 * which BIOBUZZ R704 forbids in a match. The Gradle guard is build-time only: one
 * {@code tuning=true} line in {@code gradle.properties} would put the server in every APK from then
 * on and nothing would say so (fixthese R2-A7). So the match OpModes ask at runtime.
 */
public final class BuildFlavor {
    /** A class that exists only in the tuning library. */
    public static final String TUNING_MARKER_CLASS = "com.pedropathing.tuning.autotune.Tuner";
    public static final String TUNING_WARNING =
            "!! TUNING BUILD: AutoTune web server is on. NOT match legal (R704). Rebuild without -Ptuning.";

    private static Boolean tuning = null;

    private BuildFlavor() {}

    /** True when the AutoTune library is in this APK. Cached after the first call. */
    public static synchronized boolean isTuningBuild() {
        if (tuning == null) tuning = classPresent(TUNING_MARKER_CLASS);
        return tuning;
    }

    /** Whether {@code className} can be found, without initialising it. */
    public static boolean classPresent(String className) {
        try {
            Class.forName(className, false, BuildFlavor.class.getClassLoader());
            return true;
        } catch (ClassNotFoundException | LinkageError e) {
            return false;
        }
    }
}
