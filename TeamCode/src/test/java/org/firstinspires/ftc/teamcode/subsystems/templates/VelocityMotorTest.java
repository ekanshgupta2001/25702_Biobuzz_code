package org.firstinspires.ftc.teamcode.subsystems.templates;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.FakeDcMotorEx;
import org.junit.Test;

public class VelocityMotorTest {
    private static final double EPS = 1e-9;

    @Test
    public void constructorConfiguresWithoutWriting() {
        FakeDcMotorEx motor = new FakeDcMotorEx();
        VelocityMotor vm = new VelocityMotor(motor, DcMotorSimple.Direction.REVERSE,
                DcMotor.ZeroPowerBehavior.BRAKE);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.mode);
        assertEquals(DcMotorSimple.Direction.REVERSE, motor.direction);
        assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, motor.zeroPowerBehavior);
        assertEquals(0, motor.velocityWrites);
        assertTrue(vm.isAvailable());
    }

    @Test
    public void setTargetIsIntentAndUpdateIsTheWrite() {
        FakeDcMotorEx motor = new FakeDcMotorEx();
        VelocityMotor vm = new VelocityMotor(motor);
        vm.setTarget(1500);
        assertEquals(0, motor.velocityWrites);
        vm.update();
        assertEquals(1500, motor.commandedVelocity, EPS);
        vm.write(-500);
        assertEquals("an override replaces this loop's write only", -500, motor.commandedVelocity, EPS);
        assertEquals(1500, vm.getTarget(), EPS);
    }

    @Test
    public void anUnchangedVelocityIsNotResentEveryLoop() {
        // fixthese R2-A1: six setVelocity transactions per loop for values that had not changed.
        FakeDcMotorEx motor = new FakeDcMotorEx();
        VelocityMotor vm = new VelocityMotor(motor);
        int refresh = VelocityMotor.REFRESH_EVERY_N_WRITES;
        for (int i = 0; i < refresh; i++) vm.write(1500);
        assertEquals("the first write lands, the repeats do not", 1, motor.velocityWrites);
        vm.write(1500);
        assertEquals("but an unchanged value is refreshed every REFRESH_EVERY_N_WRITES loops", 2, motor.velocityWrites);
        vm.write(-500);
        assertEquals("a change always lands", 3, motor.velocityWrites);
        assertEquals(-500, motor.commandedVelocity, EPS);
        vm.write(0);
        vm.write(0);
        assertEquals(4, motor.velocityWrites);
    }

    @Test
    public void atSpeedNeedsANonZeroTargetWithinTolerance() {
        FakeDcMotorEx motor = new FakeDcMotorEx();
        VelocityMotor vm = new VelocityMotor(motor);
        assertFalse("zero target is never at speed", vm.atSpeed(50));
        vm.setTarget(1000);
        motor.measuredVelocity = 940;
        assertFalse(vm.atSpeed(50));
        motor.measuredVelocity = 960;
        assertTrue(vm.atSpeed(50));
    }

    @Test
    public void unavailableMotorNoOps() {
        VelocityMotor vm = new VelocityMotor((DcMotorEx) null);
        assertFalse(vm.isAvailable());
        vm.setTarget(100);
        vm.update();
        assertEquals(0, vm.getVelocity(), EPS);
        assertFalse(vm.atSpeed(1000));
    }
}
