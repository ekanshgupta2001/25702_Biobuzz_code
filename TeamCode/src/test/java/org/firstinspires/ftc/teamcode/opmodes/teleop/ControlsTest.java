package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Consumer;

/**
 * The bindings against real SDK gamepads. Edge detection only advances through
 * {@code Gamepad.copy(...)}, which is how the robot controller feeds them too.
 */
public class ControlsTest {
    private static final double EPS = 1e-9;

    /** A gamepad whose state just changed to what {@code set} describes, edges included. */
    private static Gamepad after(Consumer<Gamepad> set) {
        Gamepad source = new Gamepad();
        set.accept(source);
        Gamepad pad = new Gamepad();
        pad.copy(source);
        return pad;
    }

    private static Gamepad idle() {
        return new Gamepad();
    }

    @Test
    public void everyControlHasAPadAndEdgeButtonsAreUniquePerPad() {
        for (Controls.Pad pad : Controls.Pad.values()) {
            Set<String> buttons = new HashSet<>();
            for (Controls c : Controls.values()) {
                if (c.pad() != pad || c.isAnalog()) continue;
                assertTrue(pad + " binds " + c.button() + " twice", buttons.add(c.button()));
            }
        }
        for (Controls c : Controls.values()) {
            assertNotNull(c.pad());
            assertFalse(c.button().isEmpty());
            assertFalse(c.description().isEmpty());
        }
    }

    @Test
    public void helpCardMentionsEveryControl() {
        String card = String.join("\n", Controls.helpLines());
        for (Controls c : Controls.values()) {
            assertTrue("card is missing " + c, card.contains(c.button() + "=" + c.description()));
        }
        assertTrue(card.contains("DRIVER (gamepad 1)"));
        assertTrue(card.contains("OPERATOR (gamepad 2)"));
    }

    @Test
    public void axesApplyTheSignConventionAndButtonsReadZero() {
        Gamepad driver = after(g -> {
            g.left_stick_y = -1f;
            g.left_stick_x = -1f;
            g.right_stick_x = 1f;
            g.left_trigger = 0.6f;
        });
        Gamepad operator = idle();
        assertEquals("stick up is forward", 1.0, Controls.DRIVE_FORWARD.axis(driver, operator), EPS);
        assertEquals("stick left is +strafe (Pedro: +strafe is left)", 1.0, Controls.DRIVE_STRAFE.axis(driver, operator), EPS);
        assertEquals("stick right is a clockwise, negative turn", -1.0, Controls.DRIVE_TURN.axis(driver, operator), EPS);
        assertEquals(0.6, Controls.SLOW_MODE.axis(driver, operator), 1e-6);
        assertEquals("a button has no axis", 0, Controls.RESET_HEADING.axis(driver, operator), EPS);
        assertFalse("an axis has no edge", Controls.DRIVE_FORWARD.wasPressed(driver, operator));
        assertTrue(Controls.SLOW_MODE.isAnalog());
        assertFalse(Controls.RESET_HEADING.isAnalog());
    }

    @Test
    public void buttonsRouteToTheirOwnPad() {
        Gamepad driver = idle();
        Gamepad operator = after(g -> g.y = true);
        assertFalse("Y on the operator pad is not RESET_HEADING", Controls.RESET_HEADING.wasPressed(driver, operator));
        assertTrue(Controls.INTAKE_UNTIL_FULL.wasPressed(driver, operator));
    }

    @Test
    public void readConsumesEveryEdgeExactlyOnce() {
        Gamepad driver = after(g -> g.y = true);
        Gamepad operator = after(g -> {
            g.b = true;
            g.left_trigger = 1f;
        });
        Controls.Snapshot in = Controls.read(driver, operator);
        assertTrue(in.pressed(Controls.RESET_HEADING));
        assertTrue(in.pressed(Controls.EJECT));
        assertTrue("a trigger past the threshold is an edge", in.pressed(Controls.ARM_FLYWHEEL));
        assertFalse(in.pressed(Controls.TOGGLE_DRIVE_FRAME));

        assertFalse("consumed by the snapshot", Controls.RESET_HEADING.wasPressed(driver, operator));
        assertFalse(driver.yWasPressed());
        assertFalse(Controls.read(driver, operator).pressed(Controls.RESET_HEADING));
    }

    @Test
    public void aHeldButtonFiresOnceUntilReleasedAndPressedAgain() {
        Gamepad driver = after(g -> g.y = true);
        Gamepad operator = idle();
        assertTrue(Controls.read(driver, operator).pressed(Controls.RESET_HEADING));

        Gamepad stillHeld = new Gamepad();
        stillHeld.y = true;
        driver.copy(stillHeld);
        assertFalse("held, not re-pressed", Controls.read(driver, operator).pressed(Controls.RESET_HEADING));

        driver.copy(new Gamepad());          // released
        Gamepad pressedAgain = new Gamepad();
        pressedAgain.y = true;
        driver.copy(pressedAgain);
        assertTrue(Controls.read(driver, operator).pressed(Controls.RESET_HEADING));
    }
}
