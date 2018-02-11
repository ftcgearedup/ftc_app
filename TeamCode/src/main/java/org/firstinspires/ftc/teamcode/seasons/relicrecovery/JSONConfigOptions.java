package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.JsonPrimitive;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.HashMap;
import java.util.Map;

public class JSONConfigOptions {

    private Map<String, JsonPrimitive> result = new HashMap<>();

    /**
     * This method parses the data of a JSON File to a internally stored Map<>String, Object>
     *
     * @param file the file to parse the JSON data from
     *

     */

    public void parseFile(File file){

        JsonObject obj = null;
        JsonParser parser = new JsonParser();

        try {
            obj = (JsonObject) parser.parse(new FileReader(file));
        } catch (FileNotFoundException | ClassCastException e) {

        }
        for(Map.Entry<String, JsonElement> e: obj.entrySet()){

            result.put(e.getKey(), e.getValue().getAsJsonPrimitive());

        }
    }
    /**
     * This method returns a value from the internally stored Map<>String, Object> and returns it as a JsonPrimitive value
     *     To convert the data to standard data types, place this method in parenthesis and add {@link JsonPrimitive#getAsDouble()} after it (Other data types work too. Just replace "Double" with the type you want.)
     *
     *
     *     @param key The Key of the value you want to retrieve
     *
     * @return the requested data value. Use the {@link JsonPrimitive#getAsDouble()} method to convert to any data type. Just replace "Double" with the data type of choice.
     */

    public JsonPrimitive retrieveData(String key) {
        JsonPrimitive data = null;
        if(result.containsKey(key)){
            data = this.result.get(key).getAsJsonPrimitive();
        } else {
            throw new IllegalArgumentException("Key not found: \"" + key + "\"");
        }
        return data;
    }


}