package org.frogforce503.lib.io;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerUtil {
  private TriggerUtil() {}

  /** Maps the left stick of an Xbox Controller to the left paddle. */
  public static Trigger leftPaddle(CommandXboxController controller) {
    return controller.leftStick();
  }

  /** Maps the right stick of an Xbox Controller to the right paddle. */
  public static Trigger rightPaddle(CommandXboxController controller) {
    return controller.rightStick();
  }

  /**
   * Constantly starts the given command while the button is held.
   *
   * <p>{@link Command#schedule()} will be called repeatedly while the trigger is active, and will
   * be canceled when the trigger becomes inactive.
   *
   * @param command the command to start
   * @return this trigger, so calls can be chained
   */
  public static void whileTrueContinuous(Trigger trigger, final Command command) {
    CommandScheduler.getInstance()
        .getDefaultButtonLoop()
        .bind(
            new Runnable() {
              private boolean m_pressedLast = trigger.getAsBoolean();

              @Override
              public void run() {
                boolean pressed = trigger.getAsBoolean();

                if (pressed) {
                  command.schedule();
                } else if (m_pressedLast) {
                  command.cancel();
                }

                m_pressedLast = pressed;
              }
            });
  }
}