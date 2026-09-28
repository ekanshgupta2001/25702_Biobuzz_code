package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.field.Alliance;

/** Red teleop. All the behaviour is in {@link Teleop}; this only fixes the side. */
@TeleOp(name = "Teleop RED", group = "Main")
public class RedTeleop extends Teleop {
    public RedTeleop() {
        super(Alliance.RED);
    }
}
