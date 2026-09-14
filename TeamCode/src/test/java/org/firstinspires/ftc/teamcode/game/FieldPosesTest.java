package org.firstinspires.ftc.teamcode.game;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.FieldConstants;
import org.firstinspires.ftc.teamcode.util.field.StartPosition;
import org.junit.Test;

/** The season's poses are placeholders, but they must be legal and land where the manual says. */
public class FieldPosesTest {
    private static final double EPS = 1e-6;

    @Test
    public void everyPoseAndItsRedTwinStaysOnTheField() {
        for (Pose p : FieldPoses.all()) {
            assertTrue("blue pose off field: " + p, FieldConstants.isInsideField(p));
            assertTrue("red twin off field: " + p,
                    FieldConstants.isInsideField(FieldConstants.forAlliance(p, Alliance.RED)));
        }
    }

    @Test
    public void startPoseCoversEveryStartPositionAndTheyAreDistinct() {
        Pose facing = FieldPoses.startPose(StartPosition.FACING_HIVE);
        Pose wall = FieldPoses.startPose(StartPosition.ALLIANCE_WALL);
        for (StartPosition p : StartPosition.values()) assertNotNull(FieldPoses.startPose(p));
        assertTrue("start positions must be distinct", facing.x() != wall.x() || facing.y() != wall.y());
    }

    @Test
    public void blueFacingHiveStartIsOnTheFarWallAtTheDESeam() {
        Pose blue = FieldPoses.BLUE_START_FACING_HIVE;
        assertEquals(96, blue.x(), EPS);
        assertTrue("against the far wall", blue.y() > 120 && blue.y() < 144);
        assertEquals("shooter (rear) toward the HIVE: front to the far wall", Math.toRadians(90), blue.heading(), EPS);
    }

    @Test
    public void redFacingHiveStartRotatesOntoTheAudienceWallAtTheBCSeam() {
        Pose red = FieldConstants.forAlliance(FieldPoses.BLUE_START_FACING_HIVE, Alliance.RED);
        assertEquals("B/C seam", 48, red.x(), EPS);
        assertTrue("against the audience wall", red.y() > 0 && red.y() < 24);
        assertEquals("shooter (rear) toward the HIVE: front to the audience wall", Math.toRadians(270), red.heading(), EPS);
    }

    @Test
    public void allianceWallStartsAreOnTheirOwnWallsAtMidField() {
        Pose blue = FieldPoses.BLUE_START_ALLIANCE_WALL;
        Pose red = FieldConstants.forAlliance(blue, Alliance.RED);
        assertTrue(blue.x() > 120);
        assertEquals(72, blue.y(), EPS);
        assertEquals("front to the blue wall", 0, blue.heading(), EPS);
        assertTrue(red.x() < 24);
        assertEquals(72, red.y(), EPS);
        assertEquals("front to the red wall", Math.PI, red.heading(), EPS);
    }

    @Test
    public void parkPoseOverlapsItsOwnLoadingZone() {
        double half = FieldPoses.ROBOT_HALF_LENGTH_INCHES;
        assertTrue(Field.blueLoadingZone().overlapsSquare(FieldPoses.BLUE_PARK, half));
        Pose redPark = FieldConstants.forAlliance(FieldPoses.BLUE_PARK, Alliance.RED);
        assertTrue(Field.loadingZone(Alliance.RED).overlapsSquare(redPark, half));
        assertTrue("but the robot body stays on the tiles",
                FieldPoses.BLUE_PARK.x() + half <= Field.FIELD_SIZE_INCHES);
    }

    @Test
    public void shootingSpotFacesTheFirstTargetCell() {
        Pose spot = FieldPoses.BLUE_SHOOTING_SPOT;
        Pose cell = Field.firstTargetCell(Alliance.BLUE);
        assertEquals("directly in front of the CELL", cell.x(), spot.x(), EPS);
        assertTrue("on the far side of it", spot.y() > cell.y());
        assertEquals("rear toward the CELL", Math.toRadians(90), spot.heading(), EPS);
    }
}
