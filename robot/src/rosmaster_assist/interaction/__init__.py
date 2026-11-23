"""Interaction with people: recognize hand signals and react to them.

``gestures``  body pose -> gesture (pure computation, testable without hardware)
``behavior``  gesture -> follow, come here, go away, stop (drives via the watchdog)
``voice``     voice output via the USB speaker (Piper/espeak-ng, own thread)
``speech``    voice commands via the USB microphone, wake word "Robi" (Vosk, own thread)

Overview in docs/interaction.md.
"""
