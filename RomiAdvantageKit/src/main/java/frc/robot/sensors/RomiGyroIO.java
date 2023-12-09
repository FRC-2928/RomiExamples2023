package frc.robot.sensors;

import org.littletonrobotics.junction.AutoLog;

/**
 * Connects the software to the hardware and directly receives data from the gyroscope.
 */
public interface RomiGyroIO {
    /**
     * Reads information from sources (hardware or simulation) and updates the inputs object.
     *
     * @param inputs Contains the defaults for the input values listed above.
     */
    default void updateInputs(RomiGyroIOInputs inputs) {}

    // default void getAngle(double heading) {}

    /**
     * Holds data that can be read from the corresponding gyroscope IO implementation.
     */
    @AutoLog
    class RomiGyroIOInputs {
      public double heading;
    }
}