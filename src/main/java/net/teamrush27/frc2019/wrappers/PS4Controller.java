package net.teamrush27.frc2019.wrappers;

import edu.wpi.first.wpilibj.GenericHID;

public class PS4Controller extends GenericHID {

  public PS4Controller(int port) {
    super(port);
  }

  private enum Button {
    kSquare(1),
    kX(2),
    kCircle(3),
    kTriangle(4),
    kL1(5),
    kR1(6),
    kL2(7),
    kR2(8),
    kShare(9),
    kOptions(10),
    kL3(11),
    kR3(12),
    kMiddle(13),
    kPadPress(14);

    @SuppressWarnings({"MemberName", "PMD.SingularField"})
    private final int value;

    Button(int value) {
      this.value = value;
    }
  }

  @Override
  public double getX(GenericHID.Hand hand) {
    if (hand.equals(GenericHID.Hand.kLeft)) {
      return getRawAxis(0);
    } else {
      return getRawAxis(2);
    }
  }

  @Override
  public double getY(Hand hand) {
    if (hand.equals(Hand.kLeft)) {
      return getRawAxis(1);
    } else {
      return getRawAxis(5);
    }
  }

  public double getTriggerAxis(Hand hand) {
    if (hand.equals(Hand.kLeft)) {
      return getRawAxis(3);
    } else {
      return getRawAxis(4);
    }
  }

  public boolean getBumper(Hand hand) {
    if (hand.equals(Hand.kLeft)) {
      return getRawButton(PS4Controller.Button.kL1.value);
    } else {
      return getRawButton(PS4Controller.Button.kR1.value);
    }
  }

  /**
   * Whether the bumper was pressed since the last check.
   *
   * @param hand Side of controller whose value should be returned.
   * @return Whether the button was pressed since the last check.
   */
  public boolean getBumperPressed(Hand hand) {
    if (hand == Hand.kLeft) {
      return getRawButtonPressed(PS4Controller.Button.kL1.value);
    } else {
      return getRawButtonPressed(PS4Controller.Button.kR1.value);
    }
  }

  /**
   * Whether the bumper was released since the last check.
   *
   * @param hand Side of controller whose value should be returned.
   * @return Whether the button was released since the last check.
   */
  public boolean getBumperReleased(Hand hand) {
    if (hand == Hand.kLeft) {
      return getRawButtonReleased(PS4Controller.Button.kL1.value);
    } else {
      return getRawButtonReleased(PS4Controller.Button.kR1.value);
    }
  }

  public boolean getTriggerButton(Hand hand) {
    if (hand.equals(Hand.kLeft)) {
      return getRawButton(PS4Controller.Button.kL2.value);
    } else {
      return getRawButton(PS4Controller.Button.kR2.value);
    }
  }

  public boolean getTriggerButtonPressed(Hand hand) {
    if (hand == Hand.kLeft) {
      return getRawButtonPressed(PS4Controller.Button.kL2.value);
    } else {
      return getRawButtonPressed(PS4Controller.Button.kR2.value);
    }
  }

  public boolean getTriggerButtonReleased(Hand hand) {
    if (hand == Hand.kLeft) {
      return getRawButtonReleased(PS4Controller.Button.kL2.value);
    } else {
      return getRawButtonReleased(PS4Controller.Button.kR2.value);
    }
  }

  /**
   * Read the value of the stick button on the controller.
   *
   * @param hand Side of controller whose value should be returned.
   * @return The state of the button.
   */
  public boolean getStickButton(Hand hand) {
    if (hand.equals(Hand.kLeft)) {
      return getRawButton(PS4Controller.Button.kL3.value);
    } else {
      return getRawButton(PS4Controller.Button.kR3.value);
    }
  }

  /**
   * Whether the stick button was pressed since the last check.
   *
   * @param hand Side of controller whose value should be returned.
   * @return Whether the button was pressed since the last check.
   */
  public boolean getStickButtonPressed(Hand hand) {
    if (hand == Hand.kLeft) {
      return getRawButtonPressed(PS4Controller.Button.kL3.value);
    } else {
      return getRawButtonPressed(PS4Controller.Button.kR3.value);
    }
  }

  /**
   * Whether the stick button was released since the last check.
   *
   * @param hand Side of controller whose value should be returned.
   * @return Whether the button was released since the last check.
   */
  public boolean getStickButtonReleased(Hand hand) {
    if (hand == Hand.kLeft) {
      return getRawButtonReleased(PS4Controller.Button.kL3.value);
    } else {
      return getRawButtonReleased(PS4Controller.Button.kR3.value);
    }
  }

  /**
   * Read the value of the A button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getSquareButton() {
    return getRawButton(PS4Controller.Button.kSquare.value);
  }

  /**
   * Whether the A button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getSquareButtonPressed() {
    return getRawButtonPressed(Button.kSquare.value);
  }

  /**
   * Whether the A button was released since the last check.
   *
   * @return Whether the button was released sisnce the last check.
   */
  public boolean getSquareButtonReleased() {
    return getRawButtonReleased(Button.kSquare.value);
  }

  /**
   * Read the value of the B button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getXButton() {
    return getRawButton(Button.kX.value);
  }

  /**
   * Whether the B button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getXButtonPressed() {
    return getRawButtonPressed(Button.kX.value);
  }

  /**
   * Whether the B button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getXButtonReleased() {
    return getRawButtonReleased(Button.kX.value);
  }

  /**
   * Read the value of the X button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getCircleButton() {
    return getRawButton(Button.kCircle.value);
  }

  /**
   * Whether the X button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getCircleButtonPressed() {
    return getRawButtonPressed(Button.kCircle.value);
  }

  /**
   * Whether the X button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getCircleButtonReleased() {
    return getRawButtonReleased(Button.kCircle.value);
  }

  /**
   * Read the value of the Y button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getTriangleButton() {
    return getRawButton(Button.kTriangle.value);
  }

  /**
   * Whether the Y button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getTriangleButtonPressed() {
    return getRawButtonPressed(Button.kTriangle.value);
  }

  /**
   * Whether the Y button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getTriangleReleased() {
    return getRawButtonReleased(Button.kTriangle.value);
  }

  /**
   * Read the value of the back button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getShareButton() {
    return getRawButton(Button.kShare.value);
  }

  /**
   * Whether the back button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getShareButtonPressed() {
    return getRawButtonPressed(Button.kShare.value);
  }

  /**
   * Whether the back button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getShareButtonReleased() {
    return getRawButtonReleased(Button.kShare.value);
  }

  /**
   * Read the value of the start button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getOptionsButton() {
    return getRawButton(Button.kOptions.value);
  }

  /**
   * Whether the start button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getOptionsButtonPressed() {
    return getRawButtonPressed(Button.kOptions.value);
  }

  /**
   * Whether the start button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getOptionsButtonReleased() {
    return getRawButtonReleased(Button.kOptions.value);
  }

  public boolean getMiddleButton() {
    return getRawButton(Button.kMiddle.value);
  }

  /**
   * Whether the A button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getMiddleButtonPressed() {
    return getRawButtonPressed(Button.kMiddle.value);
  }

  /**
   * Whether the A button was released since the last check.
   *
   * @return Whether the button was released sisnce the last check.
   */
  public boolean getMiddleButtonReleased() {
    return getRawButtonReleased(Button.kMiddle.value);
  }

  public boolean getPadButton() {
    return getRawButton(Button.kPadPress.value);
  }

  /**
   * Whether the A button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getPadButtonPressed() {
    return getRawButtonPressed(Button.kPadPress.value);
  }

  /**
   * Whether the A button was released since the last check.
   *
   * @return Whether the button was released sisnce the last check.
   */
  public boolean getPadButtonReleased() {
    return getRawButtonReleased(Button.kPadPress.value);
  }
}
