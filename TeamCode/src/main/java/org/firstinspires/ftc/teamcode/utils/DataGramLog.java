package org.firstinspires.ftc.teamcode.utils;

import android.util.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.io.Writer;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.text.SimpleDateFormat;
import java.util.Date;


/**
 * This class creates a custom logger class with the capabilities to
 * write to a file,
 * choose which file to write to,
 * and to clear a file of all data.
 */

public class DataGramLog {
    private InetAddress address;
    private static final int port = 6347;
    private static final String LOG_TAG = "TEAMLOG";
    private static final String FORMAT = "yyyy.MM.dd.HH.mm.ss.SS";

    public DataGramLog(InetAddress address) {
        this.address= new InetAddress(address);
    }

    /**
     * This method appends the log. It adds the date, and time, aus well as the log tag and message.
     *
     * @param text the text that you want the log message displays.
     * @param tag  the tag is attached to the log message and is used to filter log messages by type.
     */
    public void appendLog(String text, String tag) {
      appendLog(text,tag,new SimpleDateFormat(FORMAT));

    }

    /**
     * This method appends the log. It adds the date, and time, aus well as the log tag and message.
     *
     * @param text the text that you want the log message displays.
     * @param tag the tag is attached to the log message and is used to filter log messages by type.
     * @param format this allows you to chose the format for the date that you wish to call display.
     */
    public void appendLog(String text, String tag, SimpleDateFormat format) {
        String timeStamp = format.format(new Date());

        try {
            DatagramSocket socket = new DatagramSocket(this.port, this.address);

            //BufferedWriter for performance, true to set append to file flag
            StringWriter buf = new StringWriter();
            buf.append(timeStamp);
            buf.append(",");
            buf.append(tag);
            buf.append(",");
            buf.append(text);
            buf.flush();

            DatagramPacket packet = new DatagramPacket(buf.toString().getBytes(), buf.toString().getBytes().length, this.address, this.port);
            socket.send(packet);

        } catch (IOException e) {
            Log.e(LOG_TAG, Log.getStackTraceString(e));
        }

    }

    /**
     * clear the file
     **/
    public void clearLog() {
        PrintWriter writer = null;
        try {
            writer = new PrintWriter(file);
            writer.print("");
            writer.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }
}
