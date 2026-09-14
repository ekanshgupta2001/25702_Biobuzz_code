package org.firstinspires.ftc.teamcode.game;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.FieldConstants;
import org.junit.Test;

/** The field geometry is consistent with the frame it documents and with the manual's tile names. */
public class FieldTest {
    private static final double EPS = 1e-9;

    private static void assertPose(String what, double x, double y, Pose actual) {
        assertEquals(what + " x", x, actual.x(), EPS);
        assertEquals(what + " y", y, actual.y(), EPS);
    }

    @Test
    public void tileCentresFollowTheFrame() {
        assertPose("A1 (origin corner)", 12, 12, Field.tileCenter('A', 1));
        assertPose("F6 (far blue corner)", 132, 132, Field.tileCenter('F', 6));
        assertPose("C3", 60, 60, Field.tileCenter('c', 3));
        assertPose("rows increase away from the audience", 12, 36, Field.tileCenter('A', 2));
        try {
            Field.tileCenter('G', 1);
            fail("column G does not exist");
        } catch (IllegalArgumentException expected) {
            // ok
        }
    }

    @Test
    public void loadingZonesSitOnTheirTilesAndRotateOntoEachOther() {
        Field.Zone blue = Field.blueLoadingZone();
        Field.Zone red = Field.loadingZone(Alliance.RED);
        assertTrue("blue LOADING ZONE within tile F2", blue.xMin >= 120 && blue.xMax <= 144 && blue.yMin >= 24 && blue.yMax <= 48);
        assertTrue("red LOADING ZONE within tile A5", red.xMin >= 0 && red.xMax <= 24 && red.yMin >= 96 && red.yMax <= 120);
        assertEquals(Field.LOADING_ZONE_LENGTH_INCHES, blue.height(), EPS);
        assertEquals(Field.LOADING_ZONE_DEPTH_INCHES, blue.width(), EPS);
        assertEquals(144 - blue.xMax, red.xMin, EPS);
        assertEquals(144 - blue.yMax, red.yMin, EPS);
    }

    @Test
    public void gardensSitInTheCornerTiles() {
        Field.Zone blue = Field.blueGarden();
        Field.Zone red = Field.garden(Alliance.RED);
        assertTrue("blue GARDEN within F6", blue.xMin >= 120 && blue.yMin >= 120 && blue.xMax <= 144 && blue.yMax <= 144);
        assertTrue("red GARDEN within A1", red.xMin >= 0 && red.yMin >= 0 && red.xMax <= 24 && red.yMax <= 24);
    }

    @Test
    public void zoneContainmentAndOverlap() {
        Field.Zone zone = new Field.Zone(10, 30, 20, 40);
        assertTrue(zone.contains(15, 35));
        assertTrue(zone.contains(new Pose(10, 30)));
        assertTrue(!zone.contains(9.9, 35));
        assertTrue("square straddling the edge overlaps", zone.overlapsSquare(new Pose(5, 35), 6));
        assertTrue("square clear of the zone does not", !zone.overlapsSquare(new Pose(2, 35), 6));
        assertPose("centre", 15, 35, zone.center());
    }

    @Test
    public void firstTargetTagsPerAlliance() {
        assertArrayEquals(new int[] {34, 37}, Field.firstTargetTags(Alliance.RED));
        assertArrayEquals(new int[] {42, 45}, Field.firstTargetTags(Alliance.BLUE));
        assertArrayEquals(new int[] {30, 33}, Field.tagRange(Alliance.RED, Field.CellSide.FAR));
        assertArrayEquals(new int[] {38, 41}, Field.tagRange(Alliance.BLUE, Field.CellSide.AUDIENCE));
        for (Alliance a : Alliance.values()) {
            for (Field.CellSide s : Field.CellSide.values()) {
                int[] r = Field.tagRange(a, s);
                assertTrue(r[0] >= Field.MIN_TAG_ID && r[1] <= Field.MAX_TAG_ID && r[1] - r[0] == 3);
            }
        }
    }

    @Test
    public void upCellFlipsEveryTip() {
        assertEquals(Field.CellSide.FAR, Field.upCellSide(Alliance.BLUE, 0));
        assertEquals(Field.CellSide.AUDIENCE, Field.upCellSide(Alliance.BLUE, 1));
        assertEquals(Field.CellSide.FAR, Field.upCellSide(Alliance.BLUE, 2));
        assertEquals(Field.CellSide.AUDIENCE, Field.upCellSide(Alliance.RED, 0));
        assertEquals(Field.CellSide.FAR, Field.upCellSide(Alliance.RED, 1));
    }

    @Test
    public void cellsRotateOntoEachOther() {
        Pose blueFar = Field.blueCell(Field.CellSide.FAR);
        Pose redAudience = Field.cell(Alliance.RED, Field.CellSide.AUDIENCE);
        Pose rotated = FieldConstants.rotate180(blueFar);
        assertPose("red audience CELL is the rotated blue far CELL", rotated.x(), rotated.y(), redAudience);
        assertTrue("blue far CELL is on the blue side, far half", blueFar.x() > 72 && blueFar.y() > 72);
        assertTrue("red audience CELL is on the red side, audience half", redAudience.x() < 72 && redAudience.y() < 72);
        assertEquals(Field.CELL_SPACING_INCHES,
                Field.blueCell(Field.CellSide.FAR).y() - Field.blueCell(Field.CellSide.AUDIENCE).y(), EPS);
        assertPose("first blue target is the far CELL", blueFar.x(), blueFar.y(), Field.firstTargetCell(Alliance.BLUE));
        assertPose("first red target is the audience CELL", redAudience.x(), redAudience.y(), Field.firstTargetCell(Alliance.RED));
    }

    @Test
    public void everyFixedElementIsInsideTheField() {
        assertTrue(FieldConstants.isInsideField(Field.HIVE_CENTER));
        for (Pose flower : Field.flowers()) assertTrue("flower " + flower, FieldConstants.isInsideField(flower));
        for (Alliance a : Alliance.values()) {
            for (Field.CellSide s : Field.CellSide.values()) {
                assertTrue(FieldConstants.isInsideField(Field.cell(a, s)));
            }
        }
    }

    @Test
    public void flowersRotateInPairsAndEachAllianceOwnsItsWall() {
        Pose blue = Field.allianceFlower(Alliance.BLUE);
        Pose red = Field.allianceFlower(Alliance.RED);
        assertTrue("blue FLOWER on the blue wall", blue.x() > 140);
        assertTrue("red FLOWER on the red wall", red.x() < 4);
        Pose rotatedBlue = FieldConstants.rotate180(blue);
        assertPose("red FLOWER is the rotated blue FLOWER", rotatedBlue.x(), rotatedBlue.y(), red);
        Pose rotatedFar = FieldConstants.rotate180(Field.flowerFarWall());
        assertPose("audience FLOWER is the rotated far FLOWER", rotatedFar.x(), rotatedFar.y(), Field.flowerAudienceWall());
    }

    @Test
    public void driversFaceAwayFromTheirOwnWall() {
        assertEquals("red wall is -X, so red drivers face +X", 0, Field.driverForwardHeading(Alliance.RED), 1e-9);
        assertEquals("blue wall is +X, so blue drivers face -X", Math.PI, Field.driverForwardHeading(Alliance.BLUE), 1e-9);
    }

    @Test
    public void tipArithmetic() {
        assertEquals("up-CELL starts with 3 NECTAR, so 3 POLLEN tip it", 3, Field.pollenNeededForFirstTip());
        assertEquals(28, Scoring.threeShotAutoPoints());
    }
}
