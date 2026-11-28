package org.firstinspires.ftc.teamcode.opmodes.auto.limelightautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.field.Alliance;

/** Limelight auto 2 (wait for the tip, shoot, park) for the RED alliance. Everything it does, and every number to tune, is in {@link WaitForTipAuto}. */
@Autonomous(name = "LL RED: Wait for Tip", group = "Limelight", preselectTeleOp = "Teleop two driver")
public class RedWaitForTipAuto extends WaitForTipAuto {
    public RedWaitForTipAuto() {
        super(Alliance.RED);
    }
}
