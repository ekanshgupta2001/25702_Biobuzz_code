package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.StartPosition;

/**
 * Init-phase menu: dpad left/right flips the alliance, dpad up/down cycles the start position,
 * A confirms and locks, B unlocks. Every gamepad edge is read unconditionally on each poll, so a
 * press made while locked is consumed rather than latched to fire the moment B is pressed.
 */
public class AutoSelector {
    private Alliance alliance = Alliance.BLUE;
    private StartPosition start = StartPosition.FACING_HIVE;
    private boolean confirmed = false;

    /** Call once per init loop with the driver's gamepad. */
    public void poll(Gamepad gamepad) {
        boolean left = gamepad.dpadLeftWasPressed();
        boolean right = gamepad.dpadRightWasPressed();
        boolean up = gamepad.dpadUpWasPressed();
        boolean down = gamepad.dpadDownWasPressed();
        boolean confirm = gamepad.aWasPressed();
        boolean change = gamepad.bWasPressed();

        if (confirmed) {
            if (change) confirmed = false;
            return;
        }
        if (left || right) alliance = alliance.opposite();
        if (up || down) start = cycle(start, up ? -1 : 1);
        if (confirm) confirmed = true;
    }

    private static StartPosition cycle(StartPosition current, int step) {
        StartPosition[] all = StartPosition.values();
        int next = ((current.ordinal() + step) % all.length + all.length) % all.length;
        return all[next];
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public StartPosition getStart() {
        return start;
    }

    public boolean isConfirmed() {
        return confirmed;
    }

    public void unlock() {
        confirmed = false;
    }

    /** One line for the init card. */
    public String status() {
        return alliance + " / " + start.label()
                + (confirmed ? "   CONFIRMED (B to change)" : "   dpad to change, A to confirm");
    }
}
