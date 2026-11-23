"""Transport box on the robot.

Still empty. Planned: ``actuator.py`` for opening and closing the lid.

It will probably be driven by a servo on the expansion board:

    bot.set_pwm_servo(channel, angle)

The message type ``{"type": "box", "action": "open"}`` is already reserved in
the protocol; ``api/server.py`` currently only answers it with a notice.

To clarify before starting: servo or gear motor, which channel, and what
happens if something is in the way while closing.
"""
