package org.frogforce503.robot.subsystems.leds;

import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.util.Color;

public class LedsConstants {
    public static final int candleID = 11;

    // Generic requests
    public static final EmptyAnimation CLEAR_ANIMATION = new EmptyAnimation(0);
    public static final SolidColor ALL_LEDS_OFF = new SolidColor(0, 399).withColor(new RGBWColor());

    // Season-specific requests
    public static final StrobeAnimation INDEXER_FULL = new StrobeAnimation(0, 399).withColor(new RGBWColor(Color.kGreen));
    public static final StrobeAnimation READY_TO_SHOOT = new StrobeAnimation(0, 399).withColor(new RGBWColor(Color.kGreen));
    public static final SolidColor CAMERA_DISCONNECTED = new SolidColor(0, 399).withColor(new RGBWColor(Color.kRed));
}
