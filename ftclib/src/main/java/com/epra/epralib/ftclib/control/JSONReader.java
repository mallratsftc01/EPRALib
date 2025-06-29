package com.epra.epralib.ftclib.control;

import com.epra.epralib.ftclib.storage.PIDGains;
import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileReader;
import java.lang.reflect.Type;
import java.util.HashMap;
import java.util.List;

/**A class to read json files for auto.
 * <p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class JSONReader {

    private static Gson gson = new Gson();

    /**Reads objects of type T from a json.
     * @param filename The filepath of the the json.
     * @param reference A reference object of type T. This object is not used in the reading of the json.
     * @return A List of objects of type T. Null if the file can not be found.*/
    public static <T> List<T> read(String filename, T reference) {
        Type stepListType = new TypeToken<List<T>>() {}.getType();
        List<T> directions;
        File file = AppUtil.getInstance().getSettingsFile(filename);
        try (FileReader reader = new FileReader(file)) {
            directions = gson.fromJson(reader, stepListType);
        } catch (Exception e) { return null; }
        return directions;
    }

    /**Reads a json file containing pid settings.
     * @param filename The filepath of the json.
     * @return A HashMap with string ids as keys and PIDGains records as values. Null if the file can not be found. */
    public static HashMap<String, PIDGains> readPIDGains(String filename) {
        List<PIDGains> list = read(filename, new PIDGains("", 0, 0, 0));
        if (list == null) { return null; }
        HashMap<String, PIDGains> out = new HashMap<>();
        for (PIDGains p : list) {
            out.put(p.id(), p);
        }
        return out;
    }

    /**Reads filepaths from a json.
     * @param filename The filepath of the json.
     * @return An array of filepaths.*/
    public static String[] readAuto(String filename) {
        Type stringListType = new TypeToken<List<String>>() {}.getType();
        List<String> files;
        File file = AppUtil.getInstance().getSettingsFile(filename);
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
