package org.firstinspires.ftc.teamcode.utils;

import android.util.Log;

import java.io.IOException;
import java.io.StringWriter;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.text.SimpleDateFormat;
import java.util.Date;


/**
 * This class creates a custom logger class with the capabilities to
 * write to a file,
 * choose which file to write to,
 * and to clear a file of all data.
 */

public class DataGramLogServer extends Thread {
    private static final int port = 6347;
    private static final String LOG_TAG = "TEAMLOG";
    private static final String FORMAT = "yyyy.MM.dd.HH.mm.ss.SS";
    private DatagramSocket socket;
    private boolean running;
    private byte[] buf = new byte[256];

    public DataGramLogServer() throws SocketException {
        socket = new DatagramSocket(port);
    }

    public void run() {
        running = true;

        while (running) {
            DatagramPacket packet
                    = new DatagramPacket(buf, buf.length);
            try {
                socket.receive(packet);
                String received
                        = new String(packet.getData(), 0, packet.getLength());
                Log.d(LOG_TAG, "Received:" + received);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        socket.close();

    }
}