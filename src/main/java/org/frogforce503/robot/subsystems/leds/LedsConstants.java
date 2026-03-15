package org.frogforce503.robot.subsystems.leds;

import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.util.Color;

public class LedsConstants {
    // Hardware / Configuration
    public static final int candleID = 11;

    // Generic setpoints
    public static final EmptyAnimation CLEAR_ANIMATION = new EmptyAnimation(0);
    public static final SolidColor ALL_LEDS_OFF = new SolidColor(0, 399).withColor(new RGBWColor());

    // Season-specific setpoints
    public static final SolidColor SHOT_NOT_FEASIBLE = new SolidColor(0, 399).withColor(new RGBWColor(Color.kRed));
    public static final SolidColor SHOT_FEASIBLE = new SolidColor(0, 399).withColor(new RGBWColor(Color.kGreen));
}
