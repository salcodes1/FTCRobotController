package org.firstinspires.ftc.teamcode.scripting;

import org.firstinspires.ftc.teamcode.scripting.messages.HPrintMessage;

public class RVMStandardLibrary {
    public void print(String printMessage) {
        HPrintMessage message = new HPrintMessage();
        message.printMessage = printMessage;
        RVMInstance.getInstance().sendMessageToAllSockets(message);

    }
}
