package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.field.Alliance;

/** Blue teleop. All the behaviour is in {@link Teleop}; this only fixes the side. */
@TeleOp(name = "Teleop BLUE", group = "Main")
public class BlueTeleop extends Teleop {
    public BlueTeleop() {
        super(Alliance.BLUE);
    }
}
