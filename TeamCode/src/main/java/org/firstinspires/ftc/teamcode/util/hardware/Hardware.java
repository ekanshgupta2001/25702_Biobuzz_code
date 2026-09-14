package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Fail-soft hardware lookup.
 *
 * <p>{@code hardwareMap.get(...)} throws when a device is missing or misnamed in the robot
 * configuration. Because subsystems are built in {@link org.firstinspires.ftc.teamcode.Robot}'s
 * constructor, one bad name would take down the entire OpMode before it ever ran — a broken
 * colour sensor costing the whole driver-controlled period.
 *
 * <p>This helper returns {@code null} instead and records the name. Subsystems null-check the
 * handle and quietly no-op, so the robot still drives with a missing camera. {@link #getMissing()}
 * feeds the diagnostics OpMode, which reports exactly which config entry is wrong.
 *
 * <p><b>Call {@link #reset()} before constructing subsystems.</b> The registry is static (it has to
 * outlive individual subsystem constructors), so without a reset it accumulates entries across
 * OpMode runs. {@code Robot}'s constructor does this for you.
 */
public final class Hardware {
    private static final List<String> missing = new ArrayList<>();

    private Hardware() {}

    /** Clears the missing-device registry. Call once before building subsystems. */
    public static void reset() {
        missing.clear();
    }

    /**
     * Looks up a device, returning {@code null} rather than throwing when it is absent.
     *
     * @param name the name as it appears in the Robot Controller configuration
     * @return the device, or {@code null} if the configuration has no such entry
     */
    public static <T> T get(HardwareMap hardwareMap, Class<? extends T> type, String name) {
        try {
            return hardwareMap.get(type, name);
        } catch (RuntimeException missingDevice) {
            missing.add(name + " (" + type.getSimpleName() + ")");
            return null;
        }
    }

    /** Records a device that failed to initialise for a reason other than a missing config entry. */
    public static void recordFailure(String name, String reason) {
        missing.add(name + " — " + reason);
    }

    /** Names of every device that could not be resolved, in lookup order. */
    public static List<String> getMissing() {
        return Collections.unmodifiableList(missing);
    }
}
