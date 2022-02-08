package org.firstinspires.ftc.teamcode.scripting;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.teamcode.scripting.messages.BaseMessage;

import java.io.IOException;

import fi.iki.elonen.NanoHTTPD;
import fi.iki.elonen.NanoWSD;

public class RVMWebSocket extends NanoWSD.WebSocket {
    public RVMWebSocket(NanoHTTPD.IHTTPSession handshakeRequest) {
        super(handshakeRequest);
    }

    @Override
    protected void onOpen() {
        RVMInstance.getInstance().addNewSocket(this);
    }

    @Override
    protected void onClose(NanoWSD.WebSocketFrame.CloseCode code, String reason, boolean initiatedByRemote) {
        RVMInstance.getInstance().removeDeadSocket(this);
    }

    @Override
    protected void onMessage(NanoWSD.WebSocketFrame message) {
    }

    @Override
    protected void onPong(NanoWSD.WebSocketFrame pong) {

    }

    @Override
    protected void onException(IOException exception) {

    }

    public void sendMessage(BaseMessage message) throws IOException {
        JsonElement messageContent = SimpleGson.getInstance().toJsonTree(message);
        JsonObject res = new JsonObject();
        res.addProperty("op", message.getClass().getName());
        res.add("content", messageContent);

        send(res.toString());
    }
}
