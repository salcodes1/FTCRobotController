package org.firstinspires.ftc.teamcode.scripting;

import fi.iki.elonen.NanoWSD;

public class RVMWebSocketServer  extends NanoWSD {

    static final int PORT = 8000;

    public RVMWebSocketServer() {
        super(PORT);
    }

    @Override
    protected WebSocket openWebSocket(IHTTPSession handshake) {
        return new RVMWebSocket(handshake);
    }
}
