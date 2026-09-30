"""Lightweight state machine for conversation flow."""

import enum
from collections import defaultdict
from typing import Callable


class ConversationState(enum.Enum):
    IDLE = "idle"           # waiting for wake word
    LISTENING = "listening"  # recording user speech
    THINKING = "thinking"    # ASR + LLM processing
    SPEAKING = "speaking"    # TTS playback


class StateMachine:
    """Track current state and fire enter/exit callbacks on transitions.

    Usage:
        sm = StateMachine()
        sm.on_enter(ConversationState.LISTENING, start_recording)
        sm.transition(ConversationState.LISTENING)
    """

    def __init__(self):
        self._state = ConversationState.IDLE
        self._enter_callbacks: dict[ConversationState, list[Callable[[], None]]] = defaultdict(list)
        self._exit_callbacks: dict[ConversationState, list[Callable[[], None]]] = defaultdict(list)

    @property
    def state(self) -> ConversationState:
        return self._state

    def on_enter(self, state: ConversationState, callback: Callable[[], None]) -> None:
        """Register a callback to be called when entering `state`."""
        self._enter_callbacks[state].append(callback)

    def on_exit(self, state: ConversationState, callback: Callable[[], None]) -> None:
        """Register a callback to be called when leaving `state`."""
        self._exit_callbacks[state].append(callback)

    def transition(self, new_state: ConversationState) -> None:
        """Change state and fire callbacks.

        Args:
            new_state: The state to transition to.
        """
        if new_state == self._state:
            return
        old_state = self._state
        for cb in self._exit_callbacks.get(old_state, []):
            cb()
        self._state = new_state
        for cb in self._enter_callbacks.get(new_state, []):
            cb()
