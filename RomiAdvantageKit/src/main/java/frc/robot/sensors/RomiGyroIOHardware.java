package frc.robot.sensors;

public class RomiGyroIOHardware implements RomiGyroIO{
  // Set up the RomiGyro
  private final RomiGyro gyro = new RomiGyro();

  // Constructor
  public RomiGyroIOHardware() {

  }

  @Override
  public void updateInputs(RomiGyroIOInputs inputs) {       
      inputs.heading = gyro.getAngleZ();
  }
}
