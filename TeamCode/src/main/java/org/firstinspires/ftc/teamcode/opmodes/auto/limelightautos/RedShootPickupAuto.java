package org.firstinspires.ftc.teamcode.opmodes.auto.limelightautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.field.Alliance;

/** Limelight auto 1 (shoot, pick up, shoot again if not tipped) for the RED alliance. Everything it does, and every number to tune, is in {@link ShootPickupAuto}. */
@Autonomous(name = "LL RED: Shoot + Pickup", group = "Limelight", preselectTeleOp = "Teleop two driver")
public class RedShootPickupAuto extends ShootPickupAuto {
    public RedShootPickupAuto() {
        super(Alliance.RED);
    }
}
