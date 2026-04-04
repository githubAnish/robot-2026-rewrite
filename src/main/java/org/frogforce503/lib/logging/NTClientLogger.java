package org.frogforce503.lib.logging;

import java.nio.ByteBuffer;
import java.util.HashSet;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Utility class to log the list of NetworkTables clients. */
public final class NTClientLogger {
    private static final String tableName = "NTClients/";
    private static Set<String> lastRemoteIds = new HashSet<>();
    private static ByteBuffer intBuffer = ByteBuffer.allocate(4);

    private NTClientLogger() {}

    public static void log() {
        ConnectionInfo[] connections = NetworkTableInstance.getDefault().getConnections();
        Set<String> remoteIds = new HashSet<>();

        // Log data for connected clients
        for (ConnectionInfo conn : connections) {
            String id = conn.remote_id;
            String key = tableName + id + "/";

            lastRemoteIds.remove(id);
            remoteIds.add(id);

            Logger.recordOutput(key + "Connected", true);
            Logger.recordOutput(key + "IPAddress", conn.remote_ip);
            Logger.recordOutput(key + "RemotePort", conn.remote_port);
            Logger.recordOutput(key + "LastUpdate", conn.last_update);
            
            intBuffer.rewind();

            Logger.recordOutput(
                key + "ProtocolVersion",
                intBuffer.putInt(conn.protocol_version).array());
        }

        // Mark disconnected clients
        for (var remoteId : lastRemoteIds) {
            Logger.recordOutput(tableName + remoteId + "/Connected", false);
        }

        lastRemoteIds = remoteIds;
    }
}