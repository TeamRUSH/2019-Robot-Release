package net.teamrush27.frc2019.wrappers;

import edu.wpi.first.wpilibj.DigitalInput;

public class InvertableDigitalInput extends DigitalInput {

  private final boolean inverted;

  /**
   * Create an instance of a Digital Input class. Creates a digital input given a channel.
   *
   * @param channel the DIO channel for the digital input 0-9 are on-board, 10-25 are on the MXP
   */
  public InvertableDigitalInput(int channel) {
    super(channel);
    inverted = false;
  }

  public InvertableDigitalInput(int channel, boolean inverted) {
    super(channel);
    this.inverted = inverted;
  }

  @Override
  public boolean get() {
    return inverted ^ super.get();
  }
}
