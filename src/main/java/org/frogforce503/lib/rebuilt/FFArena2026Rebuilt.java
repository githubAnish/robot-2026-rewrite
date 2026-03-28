package org.frogforce503.lib.rebuilt;

import org.frogforce503.lib.math.AllianceFlipUtil;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FFArena2026Rebuilt extends Arena2026Rebuilt {
    private final double fuelDiameter = Units.inchesToMeters(5.91);

    public FFArena2026Rebuilt(boolean AddRampCollider) {
        super(AddRampCollider);
    }

    @Override
    public void placeGamePiecesOnField() {
        blueOutpost.reset();
        redOutpost.reset();

        // Add depot fuel
        for (int x = 0; x < 4; x++) {
            for (int y = 0; y < 6; y++) {
                Translation2d blueFuelPosition =
                    FieldConstants.Depot.blue
                        .getBackLeftCorner()
                        .plus(new Translation2d(fuelDiameter / 2, -(fuelDiameter + Units.inchesToMeters(0.5)))) // bottom left corner to bottom left fuel offset
                        .plus(new Translation2d(fuelDiameter * x, -fuelDiameter * y));

                Translation2d redFuelPosition = AllianceFlipUtil.mirror(AllianceFlipUtil::apply, blueFuelPosition);

                SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(blueFuelPosition));
                SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(redFuelPosition));
            }
        }

        // Add NZ fuel
        for (int x = 0; x < 12; x++) {
            for (int y = 0; y < 30; y += 2) {
                addGamePiece(
                    new RebuiltFuelOnField(
                        centerPieceBottomRightCorner
                            .plus(new Translation2d(Units.inchesToMeters(5.991 * x), Units.inchesToMeters(5.95 * y)))));
            }
        }

        setupValueForMatchBreakdown("CurrentFuelInOutpost");
        setupValueForMatchBreakdown("TotalFuelInOutpost");
        setupValueForMatchBreakdown("TotalFuelInHub");
        setupValueForMatchBreakdown("WastedFuel");
    }
}
