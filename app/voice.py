"""voice.py -- spoken audio alerts (QGC-style "audio annunciation").

When you fly, your eyes are on the aircraft, not the GCS screen. QGroundControl speaks the critical
events aloud; DroneDeck now can too. VoiceAlerts wraps Qt's text-to-speech (the speech-dispatcher engine
+ espeak-ng) and announces arm/disarm, mode changes, and -- driven from the existing edge-triggered
annunciators in main -- low/critical battery and failsafe. It is OFF by default (a talking GCS should be
opt-in) and degrades silently when no speech backend is installed. All edge detection lives here so it is
unit-testable without a running Qt speech engine (inject a capture function as `speak`)."""
from __future__ import annotations

try:
    from PySide6.QtTextToSpeech import QTextToSpeech
    _HAVE_TTS = True
except Exception:                       # pragma: no cover - depends on the host
    _HAVE_TTS = False


class VoiceAlerts:
    def __init__(self, speak=None, enabled=False, engine="speechd"):
        self.enabled = bool(enabled)
        self._ext_speak = speak         # test/override hook: callable(str)
        self._tts = None
        self._prev = {}                 # sysid -> {"armed": bool, "mode": str}
        if speak is None and _HAVE_TTS:
            try:
                tts = QTextToSpeech(engine)
                if tts.availableVoices():      # a backend with no voices can't actually speak
                    self._tts = tts
            except Exception:                  # pragma: no cover - host dependent
                self._tts = None

    @property
    def available(self) -> bool:
        return self._ext_speak is not None or self._tts is not None

    def set_enabled(self, on: bool):
        self.enabled = bool(on)

    def say(self, text):
        """Speak `text` now, interrupting any in-progress utterance (a fresh alert matters more than a
        stale one). No-op unless enabled AND a backend is available."""
        if not (self.enabled and text):
            return
        if self._ext_speak is not None:
            self._ext_speak(text)
        elif self._tts is not None:            # pragma: no cover - needs a live speech engine
            self._tts.say(text)

    def update(self, ve):
        """Announce arm/disarm and mode changes for a vehicle. Edge-triggered per sysid; the FIRST
        update for a vehicle only records state (no announcement of the initial condition)."""
        sid = getattr(ve, "sysid", 0)
        if not sid:
            return
        armed = bool(getattr(ve, "armed", False))
        mode = getattr(ve, "mode", None)
        p = self._prev.get(sid)
        if p is not None:
            if armed != p["armed"]:
                self.say("Armed" if armed else "Disarmed")
            if mode != p["mode"] and mode not in (None, "", "--"):
                self.say(f"Mode {mode}")
        self._prev[sid] = {"armed": armed, "mode": mode}

    def reset(self):
        """Forget per-vehicle state (e.g. on disconnect) so a reconnect doesn't announce stale edges."""
        self._prev.clear()
