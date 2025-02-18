package org.firstinspires.ftc.teamcode.common.commandbase.command.state;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.Bot;

public class SetGamepadLight extends CommandBase {
    GamepadEx gamepadEx;
    Bot bot;

    public SetGamepadLight(Bot bot,GamepadEx gamepadEx){
        this.gamepadEx = gamepadEx;
        this.bot = bot;
    }

    @Override
    public void initialize(){
        if (bot.getMode() == Bot.Modes.SAMPLES) {
            gamepadEx.gamepad.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepadEx.gamepad.setLedColor(0,0,255, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }
}
