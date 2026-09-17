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
    public void loadingZoneCentreSitsOnTileF2AgainstTheBlueWallAndRotatesOntoA5() {
        Pose blue = Field.blueLoadingZoneCenter();
        assertTrue("blue LOADING ZONE within tile F2, against the wall",
                blue.x() > 120 && blue.x() < 144 && blue.y() >= 24 && blue.y() <= 48);
        Pose red = FieldConstants.forAlliance(blue, Alliance.RED);
        assertTrue("red LOADING ZONE within tile A5", red.x() > 0 && red.x() < 24 && red.y() >= 96 && red.y() <= 120);
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
        for (Alliance a : Alliance.values()) {
            for (Field.CellSide s : Field.CellSide.values()) {
                assertTrue(FieldConstants.isInsideField(Field.cell(a, s)));
            }
        }
    }

    @Test
    public void driversFaceAwayFromTheirOwnWall() {
        assertEquals("red wall is -X, so red drivers face +X", 0, Field.driverForwardHeading(Alliance.RED), 1e-9);
        assertEquals("blue wall is +X, so blue drivers face -X", Math.PI, Field.driverForwardHeading(Alliance.BLUE), 1e-9);
    }
}
