package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * {@link HardcodedAuto} without the shooting: at START it drives forward, stops, and drives back,
 * with HardcodedAuto's DRIVE_ constants, so tuning one tunes both. The shooter never spins and the
 * pre-loaded balls stay on the robot.
 */
@Autonomous(name = "Auto HARDCODED NO SHOOT", group = "Main", preselectTeleOp = "Teleop")
public class HardcodedNoShootAuto extends HardcodedAuto {
    public HardcodedNoShootAuto() {
        super(false);
    }
}
