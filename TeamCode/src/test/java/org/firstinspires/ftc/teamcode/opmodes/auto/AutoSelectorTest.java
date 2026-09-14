package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.StartPosition;
import org.junit.Test;

import java.util.function.Consumer;

/** The init menu against a real SDK gamepad driven by copy(), the way the robot controller feeds it. */
public class AutoSelectorTest {
    private final Gamepad pad = new Gamepad();
    private final AutoSelector selector = new AutoSelector();

    private void press(Consumer<Gamepad> set) {
        Gamepad source = new Gamepad();
        set.accept(source);
        pad.copy(source);
        selector.poll(pad);
        pad.copy(new Gamepad());   // release
        selector.poll(pad);
    }

    @Test
    public void defaultsToBlueFacingHiveUnconfirmed() {
        assertEquals(Alliance.BLUE, selector.getAlliance());
        assertEquals(StartPosition.FACING_HIVE, selector.getStart());
        assertFalse(selector.isConfirmed());
        assertTrue(selector.status().contains("A to confirm"));
    }

    @Test
    public void dpadFlipsAllianceAndCyclesStart() {
        press(g -> g.dpad_left = true);
        assertEquals(Alliance.RED, selector.getAlliance());
        press(g -> g.dpad_right = true);
        assertEquals(Alliance.BLUE, selector.getAlliance());
        press(g -> g.dpad_down = true);
        assertEquals(StartPosition.ALLIANCE_WALL, selector.getStart());
        press(g -> g.dpad_down = true);
        assertEquals("wraps around", StartPosition.FACING_HIVE, selector.getStart());
        press(g -> g.dpad_up = true);
        assertEquals(StartPosition.ALLIANCE_WALL, selector.getStart());
    }

    @Test
    public void aLocksAndBUnlocksWithoutReplayingPresses() {
        press(g -> g.dpad_left = true);
        press(g -> g.a = true);
        assertTrue(selector.isConfirmed());
        assertTrue(selector.status().contains("CONFIRMED"));

        press(g -> g.dpad_left = true);           // ignored while locked, and consumed
        assertEquals(Alliance.RED, selector.getAlliance());

        press(g -> g.b = true);
        assertFalse(selector.isConfirmed());
        assertEquals("the press made while locked did not fire on unlock", Alliance.RED, selector.getAlliance());
    }
}
