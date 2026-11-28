package org.firstinspires.ftc.teamcode.opmodes.auto.limelightautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.field.Alliance;

/** Limelight auto 2 (wait for the tip, shoot, park) for the BLUE alliance. Everything it does, and every number to tune, is in {@link WaitForTipAuto}. */
@Autonomous(name = "LL BLUE: Wait for Tip", group = "Limelight", preselectTeleOp = "Teleop two driver")
public class BlueWaitForTipAuto extends WaitForTipAuto {
    public BlueWaitForTipAuto() {
        super(Alliance.BLUE);
    }
}
