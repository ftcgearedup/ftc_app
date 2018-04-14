package org.firstinspires.ftc.teamcode.utils;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

import java.io.IOException;
import java.io.StringWriter;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.text.SimpleDateFormat;
import java.util.Date;


public class DataGramLogServer extends Thread {
    private static final int port = 6347;
    private static final String LOG_TAG = "TEAMLOG";
    private static final String FORMAT = "yyyy.MM.dd.HH.mm.ss.SS";
    private DatagramSocket socket;
    private boolean running;
    private byte[] buf = new byte[256];
 private OpMode opMode;
    public DataGramLogServer(OpMode opMode) throws SocketException {
        socket = new DatagramSocket(port);
        this.opMode= opMode;
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
                opMode.telemetry.addData("Computer IP address =", packet.getAddress().getHostAddress());
                opMode.telemetry.update();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        socket.close();

    }
}