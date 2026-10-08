package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.field.Alliance;

/** Blue autonomous. All the behaviour is in {@link Auto}; this only fixes the side. */
@Autonomous(name = "Auto BLUE", group = "Main", preselectTeleOp = "Teleop")
public class BlueAuto extends Auto {
    public BlueAuto() {
        super(Alliance.BLUE);
    }
}
