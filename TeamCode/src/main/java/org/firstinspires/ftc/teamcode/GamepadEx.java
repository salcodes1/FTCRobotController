package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.HashMap;

public class GamepadEx {

	private com.qualcomm.robotcore.hardware.Gamepad gamepad;


	/*
	Digital input:
	0	START
	1	BACK
	2	MODE
	3	A
	4	B
	5	X
	6	Y
	7	DPAD_UP
	8	DPAD_RIGHT
	9	DPAD_DOWN
	10	DPAD_LEFT
	11	BUMPER_LEFT
	12	BUMPER_RIGHT
	13	JOYSTICK_LEFT
	14	JOYSTICK_RIGHT
	Analog input:
	0	JOYSTICK_LEFT_X
	1	JOYSTICK_LEFT_Y
	2	JOYSTICK_RIGHT_X
	3	JOYSTICK_RIGHT_Y
	4	TRIGGER_LEFT
	5	TRIGGER_RIGHT
	*/
	private boolean[] buttonState = new boolean[15];
	private boolean[] buttonDown = new boolean[15];
	private boolean[] buttonUp = new boolean[15];

	private float[] analog = new float[6];

	// TODO update to java 8 way when possible
	public static final HashMap<String, Integer> buttonName = new HashMap<String, Integer>() {{
		put("start", 0);
		put("back", 1);
		put("mode", 2);
		put("a", 3);
		put("b", 4);
		put("x", 5);
		put("y", 6);
		put("dpad_up", 7);
		put("dpad_right", 8);
		put("dpad_down", 9);
		put("dpad_left", 10);
		put("bumper_left", 11);
		put("bumper_right", 12);
		put("joystick_left", 13);
		put("joystick_right", 14);
	}};

	public static final HashMap<String, Integer> analogName = new HashMap<String, Integer>() {{
		put("left_x", 0);
		put("left_y", 1);
		put("right_x", 2);
		put("right_y", 3);
		put("left_trigger", 4);
		put("right_trigger", 5);

	}};

	public GamepadEx(com.qualcomm.robotcore.hardware.Gamepad gamepad)
	{
		this.gamepad = gamepad;
		update();
	}

	public void update()
	{
		analog[0]	= gamepad.left_stick_x;
		analog[1]	= gamepad.left_stick_y;
		analog[2]	= gamepad.right_stick_x;
		analog[3]	= gamepad.right_stick_y;
		analog[4]	= gamepad.left_trigger;
		analog[5]	= gamepad.right_trigger;

		boolean[] newButtonState = new boolean[15];

		newButtonState[0] 	= gamepad.start;
		newButtonState[1] 	= gamepad.back;
		newButtonState[2] 	= false;
		// TODO find better way to ignore start
		newButtonState[3] 	= gamepad.a & !newButtonState[0]; // ignore start + a
		newButtonState[4] 	= gamepad.b & !newButtonState[0]; // ignore start + b
		newButtonState[5] 	= gamepad.x;
		newButtonState[6] 	= gamepad.y;
		newButtonState[7] 	= gamepad.dpad_up;
		newButtonState[8] 	= gamepad.dpad_right;
		newButtonState[9] 	= gamepad.dpad_down;
		newButtonState[10] 	= gamepad.dpad_left;
		newButtonState[11] 	= gamepad.left_bumper;
		newButtonState[12] 	= gamepad.right_bumper;
		newButtonState[13] 	= gamepad.left_stick_button;
		newButtonState[14] 	= gamepad.right_stick_button;

		for(int i = 0; i < 15; i++)
		{
			buttonDown[i] = newButtonState[i] && (newButtonState[i] != buttonState[i]);
			buttonUp[i] = (!newButtonState[i]) && (newButtonState[i] != buttonState[i]);
		}
		buttonState = newButtonState;
	}

	public float getAnalog(String analog) {return this.analog[analogName.get(analog)];}

	public boolean getButton(String button)
	{
		return buttonState[buttonName.get(button)];
	}

	public boolean getButtonDown(String button)
	{
		return buttonDown[buttonName.get(button)];

	}

	public boolean getButtonUp(String button)
	{
		return buttonUp[buttonName.get(button)];
	}
}