package com.epra.epralib.ftclib.control;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileReader;
import java.lang.reflect.Type;
import java.util.List;

/**A class to read json files for auto.
 * <p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class JSONReader {

    private static Gson gson = new Gson();

    /**Reads auto steps from a json.
     * @param fileName The filepath of the the json.
     * @return An array of auto steps.*/
    public static Step[] readSteps(String fileName) {
        Type stepListType = new TypeToken<List<Step>>() {}.getType();
        List<Step> directions;
        File file = AppUtil.getInstance().getSettingsFile(fileName);
        try (FileReader reader = new FileReader(file)) {
            directions = gson.fromJson(reader, stepListType);
        } catch (Exception e) { return new Step[1]; }
        Step[] r = new Step[directions.size()];
        for (int i = 0; i < r.length; i++) {
            r[i] = directions.get(i);
        }
        return r;
    }

    /**Reads filepaths from a json.
     * @param fileName The filepath of the json.
     * @return An array of filepaths.*/
    public static String[] readAuto(String fileName) {
        Type stringListType = new TypeToken<List<String>>() {}.getType();
        List<String> files;
        File file = AppUtil.getInstance().getSettingsFile(fileName);
        try (FileReader reader = new FileReader(file)) {
            files = gson.fromJson(reader, stringListType);
        } catch (Exception e) { return new String[0]; }
        if (files.isEmpty()) { return new String[]{"list_empty"}; }
        String[] r = new String[files.size()];
        for (int i = 0; i < r.length; i++) {
            r[i] = files.get(i);
        }
        return r;
    }
}
