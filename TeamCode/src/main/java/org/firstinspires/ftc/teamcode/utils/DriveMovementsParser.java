package org.firstinspires.ftc.teamcode.utils;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import com.google.gson.stream.JsonReader;

import org.firstinspires.ftc.teamcode.mechanism.drivetrain.DriveMovement;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.List;
import java.util.Stack;

/**
 *
 */

public class DriveMovementsParser {
    private Gson gson;

    /**
     *
     */
    public DriveMovementsParser() {
        this.gson = new Gson();
    }

    /**
     *
     * @param configFile
     * @return
     * @throws FileNotFoundException
     */
    public Stack<DriveMovement> parseDriveMovements(File configFile) throws FileNotFoundException {
        JsonReader reader = new JsonReader(new FileReader(configFile));
        return gson.fromJson(reader, new TypeToken<Stack<DriveMovement>>(){}.getType());
    }
}
