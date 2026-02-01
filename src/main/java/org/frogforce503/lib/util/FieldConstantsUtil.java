package org.frogforce503.lib.util;

import java.io.FileReader;
import java.io.IOException;

import org.frogforce503.robot.Constants;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public final class FieldConstantsUtil {
    private static JSONObject fieldJson;

    private FieldConstantsUtil() {}

    static {
        try {
            fieldJson =
                (JSONObject) new JSONParser()
                    .parse(
                        new FileReader(
                            Filesystem.getDeployDirectory().getAbsolutePath() + "/fields/" + Constants.fieldVenue.getFilePath()));
        } catch (IOException | ParseException e) {
            System.out.println("Error finding / reading field json" + ErrorUtil.attachJavaClassName(FieldConstantsUtil.class));
            e.printStackTrace();
        }
    }

    public static String jsonEntryToString(String key) {
        return fieldJson.get(key).toString();
    }

    /** <b> Make sure there is only 1 value associated with the {@code key}. </b> */
    public static boolean jsonEntryToBool(String key) {
        return Boolean.parseBoolean(jsonEntryToString(key));
    }

    /** <b> Make sure there is only 1 value associated with the {@code key}. </b> */
    public static double jsonEntryToInt(String key) {
        return Integer.parseInt(jsonEntryToString(key));
    }

    /** <b> Make sure there is only 1 value associated with the {@code key}. </b> */
    public static double jsonEntryToDouble(String key) {
        return Double.parseDouble(jsonEntryToString(key));
    }

    /**
     * <p> Gets the value associated with {@code key} and converts it from inches to meters. </p>
     * <p> Make sure <b> there is only 1 value associated with the {@code key}. </b></p>
     * <p> Make sure the value associated with {@code key} is in <b> inches in the field json </b> file. </p>
     */
    public static double getFieldValueMeters(String key) {
        return Units.inchesToMeters(jsonEntryToDouble(key));
    }
}
