package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class OI {
    GamepadEx driver, operator;

    public OI(GamepadEx driver, GamepadEx operator){
        this.driver = driver;
        this.operator = operator;
    }

    public Button shooterButton(){
        return new GamepadButton(
                driver, GamepadKeys.Button.RIGHT_BUMPER
        );
    }

    public boolean toggleMotif(){
        return operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER);
    }


}
