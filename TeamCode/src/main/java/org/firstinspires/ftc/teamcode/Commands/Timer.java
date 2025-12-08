package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import java.util.concurrent.TimeUnit;

public class Timer extends CommandBase {
    Timing.Timer timer;
    public Timer(Long time){
        timer = new Timing.Timer(time, TimeUnit.MILLISECONDS);
    }

    @Override
    public void initialize() {
        super.initialize();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.done();

    }
}
