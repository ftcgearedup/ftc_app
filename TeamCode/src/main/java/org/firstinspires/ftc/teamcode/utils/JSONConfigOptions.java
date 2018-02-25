package org.firstinspires.ftc.teamcode.utils;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.HashMap;
import java.util.Map;

public class JSONConfigOptions {

    public JSONConfigOptions(String fileName){
        parseFile(new File(AppUtil.FIRST_FOLDER + "/" + fileName));
    }

    private Map<String, JsonElement> result = new HashMap<>();

    private void parseFile(File file){

        JsonObject obj = null;
        JsonParser parser = new JsonParser();

        try {
            obj = (JsonObject) parser.parse(new FileReader(file));
        } catch (FileNotFoundException | ClassCastException e) {

        }
        for(Map.Entry<String, JsonElement> e: obj.entrySet()){

            result.put(e.getKey(), e.getValue());

        }
    }

    /**
     * This method returns a value from the internally stored Map<>String, JsonElement> and returns it as an int.
     * @param key the name of the key
     * @return the requested data type as an int
     */
    public int retrieveAsInt(String key){
        return (int) retrieveData(key, Integer.class);
    }

    /**
     * This method returns a value from the internally stored Map<>String, JsonElement> and returns it as a double.
     * @param key the name of the key
     * @return the requested data type as a double
     */
    public double retrieveAsDouble(String key){
        return (double) retrieveData(key, Double.class);
    }

    /**
     * This method returns a value from the internally stored Map<>String, JsonElement> and returns it as a boolean.
     * @param key the name of the key
     * @return the requested data type as a boolean
     */
    public boolean retrieveAsBoolean(String key){
        return (boolean) retrieveData(key, Boolean.class);
    }

    /**
     * This method returns a value from the internally stored Map<>String, JsonElement> and returns it as a String.
     * @param key the name of the key
     * @return the requested data type as a String
     */
    public String retrieveAsString(String key){
        return (String) retrieveData(key, String.class);
    }

    private Object retrieveData(String key, Class<?> type) {
        JsonElement value;
        Object obj = null;
        if (result.containsKey(key)) {
            value = result.get(key);
            if (type.equals(Double.class)) {
                obj = value.getAsDouble();
            } else if (type.equals(Integer.class)) {
                obj = value.getAsInt();
            } else if (type.equals(Boolean.class)) {
                obj = value.getAsBoolean();
            } else if (type.equals(String.class)) {
                obj = value.getAsString();
            }
        } else {
            throw new IllegalArgumentException("Key not found: \"" + key + "\"");
        }
        if(obj == null){
            throw new IllegalStateException("Type is not a JSON Primitive: \"" + type.toString() + "\"");
        }
        return obj;
    }
}