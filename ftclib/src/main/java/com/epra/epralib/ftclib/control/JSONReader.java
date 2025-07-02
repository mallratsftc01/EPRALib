package com.epra.epralib.ftclib.control;

import com.epra.epralib.ftclib.storage.PIDGains;
import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileReader;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**A class to read json files for auto.
 * <p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class JSONReader {

    private static Gson gson = new Gson();

    /**Reads objects of type T from a json.
     * @param filename The filepath of the the json.
     * @param directions The ArrayList the objects from the json will be added to.
     * @param listType The Type of the list (use new TypeToken<List<T>>() {}.getType())
     * @return A List of objects of type T. Null if the file can not be found.*/
    public static void read(String filename, List<?> directions, Type listType) {
        File file = AppUtil.getInstance().getSettingsFile(filename);
        try (FileReader reader = new FileReader(file)) {
            directions.addAll(gson.fromJson(reader, listType));
        } catch (Exception e) { }
    }

    /**Reads a json file containing pid settings.
     * @param filename The filepath of the json.
     * @return A HashMap with string ids as keys and PIDGains records as values. Null if the file can not be found. */
    public static HashMap<String, PIDGains> readPIDGains(String filename) {
        ArrayList<PIDGains> list = new ArrayList<>();
        read(filename, list, new TypeToken<List<PIDGains>>() {}.getType());
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
