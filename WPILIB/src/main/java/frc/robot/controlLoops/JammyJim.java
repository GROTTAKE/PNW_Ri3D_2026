package frc.robot.controlLoops;

import java.time.Duration;

import frc.robot.utils.ElapsedTimer;

public class JammyJim<T extends Comparable<T>>
{
    public final T jamThreshold;
    private final ElapsedTimer jamTimer;
    private boolean jamActive = false;

    public JammyJim(T jamThreshold, Duration jamFixDuration)
    {
        this.jamThreshold = jamThreshold;
        this.jamTimer = new ElapsedTimer(jamFixDuration);
    }

    public JammyResults update(T measure)
    {
        boolean jamDetected = measure.compareTo(jamThreshold) > 0;

        if (jamDetected && !jamActive)
        {
            jamActive = true;
            jamTimer.Timer.restart();
        }

        if (jamActive)
        {
            if (jamTimer.hasElapsed())
            {
                jamActive = false;
                jamTimer.Timer.stop();

                if (jamDetected)
                {
                    return JammyResults.Failed;
                }
                else
                {
                    return JammyResults.Unjammed;
                }
            }

            return JammyResults.Unjamming;
        }

        return JammyResults.Ok;
    }

    public enum JammyResults
    {
        Ok,
        Unjamming,
        Unjammed,
        Failed
    }
}
