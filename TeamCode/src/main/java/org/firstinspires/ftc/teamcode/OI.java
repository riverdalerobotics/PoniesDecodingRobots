package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
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
                driver, GamepadKeys.Button.A
        );
    }

    public boolean toggleMotif(){
        return operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER);
    }
    public Button revButton(){
        return new GamepadButton(
                driver, GamepadKeys.Button.RIGHT_BUMPER
        );
    }
    public Button pointAtAT(){
        return new GamepadButton(
                driver, GamepadKeys.Button.LEFT_BUMPER
        );
    }
    public Button closeShot(){
        return new GamepadButton(
                driver, GamepadKeys.Button.X
        );
    }
    public Button farShot(){
        return new GamepadButton(
                driver, GamepadKeys.Button.RIGHT_BUMPER
        );
    }
   public boolean shoot(){
        return operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.2;
   }


}
