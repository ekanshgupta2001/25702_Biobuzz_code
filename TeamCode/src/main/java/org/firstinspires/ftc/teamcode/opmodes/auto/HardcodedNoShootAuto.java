package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * {@link HardcodedAuto} without the shooting: at START it strafes left, drives forward and strafes
 * right, with HardcodedAuto's drive constants, so tuning one tunes both. The flywheel never spins and
 * the pre-loaded balls stay on the robot.
 */
@Autonomous(name = "Auto HARDCODED NO SHOOT", group = "Main", preselectTeleOp = "Teleop")
public class HardcodedNoShootAuto extends HardcodedAuto {
    public HardcodedNoShootAuto() {
        super(false);
    }
}
