package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.field.Alliance;

/** Red autonomous. All the behaviour is in {@link Auto}; this only fixes the side. */
@Autonomous(name = "Auto RED", group = "Main", preselectTeleOp = "Teleop")
public class RedAuto extends Auto {
    public RedAuto() {
        super(Alliance.RED);
    }
}
