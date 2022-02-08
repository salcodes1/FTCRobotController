package org.firstinspires.ftc.teamcode.scripting;

import org.firstinspires.ftc.teamcode.scripting.messages.BaseMessage;
import org.mozilla.javascript.Context;
import org.mozilla.javascript.Function;
import org.mozilla.javascript.Scriptable;

import java.io.IOException;
import java.util.ArrayList;

public class RVMInstance {
    static RVMInstance instance;

    Context ctx;
    Scriptable scope;

    RVMWebSocketServer webSocketServer;
    ArrayList<RVMWebSocket> activeWebSockets = new ArrayList<>();


    RVMInstance() {
        // Scripting backend init
        ctx = Context.enter();
        ctx.setOptimizationLevel(-1); // needed for android
        scope = ctx.initStandardObjects();

        /// ?? should call Context.close() if the class is static ??

        RVMStandardLibrary standardLibrary = new RVMStandardLibrary();
        scope.put("std", scope, standardLibrary);

        // Websocket init

        webSocketServer = new RVMWebSocketServer();

    }


    public static RVMInstance getInstance()
    {
        if(instance == null) instance = new RVMInstance();

        return instance;
    }

    public void runScript(String script) {
        ctx.evaluateString(scope, script, "script", 0, null);

        runFunction("init", new Object[]{});


        if(script.startsWith("// AUTO")) {
            runFunction("auto", new Object[]{});
        } else if(script.startsWith("// TELEOP")) {
            runFunction("teleop", new Object[]{});
        }
    }

    void runFunction(String name, Object[] params) {
        Object func = scope.get(name, scope);

        if(func == Scriptable.NOT_FOUND) {
            // error out
        } else if(!(func instanceof Function)){
            // error out
        } else {
            ((Function) func).call(ctx, scope, scope, params);
        }
    }

    public void sendMessageToAllSockets(BaseMessage message) {
        for (RVMWebSocket socket : activeWebSockets) {
            try {
                socket.sendMessage(message);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public void addNewSocket(RVMWebSocket socket) {
        activeWebSockets.add(socket);
    }

    public void removeDeadSocket(RVMWebSocket socket) {
        activeWebSockets.remove(socket);
    }
}
