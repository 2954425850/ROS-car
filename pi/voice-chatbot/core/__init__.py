"""Core orchestration — state machine and conversation flow."""
from core.state_machine import StateMachine, ConversationState

__all__ = ["StateMachine", "ConversationState", "ConversationManager"]


def __getattr__(name):
    if name == "ConversationManager":
        from core.conversation import ConversationManager
        return ConversationManager
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
