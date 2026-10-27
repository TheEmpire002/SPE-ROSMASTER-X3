"""Transportbox auf dem Roboter.

Noch leer. Geplant: ``actuator.py`` zum Öffnen und Schließen des Deckels.

Angesteuert wird vermutlich ein Servo am Expansion Board:

    bot.set_pwm_servo(kanal, winkel)

Der Nachrichtentyp ``{"type": "box", "action": "open"}`` ist im Protokoll
bereits vorgesehen; ``api/server.py`` antwortet darauf aktuell nur mit einem
Hinweis.

Zu klären, bevor es losgeht: Servo oder Getriebemotor, welcher Kanal, und was
passiert, wenn beim Schließen etwas im Weg ist.
"""
