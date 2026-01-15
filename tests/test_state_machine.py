from robertito.state_machine import AssistantState, AssistantStateMachine


def test_state_machine_transitions():
    sm = AssistantStateMachine()
    assert sm.current() == AssistantState.IDLE

    prev = sm.transition(AssistantState.LISTENING)
    assert prev == AssistantState.IDLE
    assert sm.current() == AssistantState.LISTENING

    sm.transition(AssistantState.SPEAKING)
    assert sm.current() == AssistantState.SPEAKING

    sm.reset()
    assert sm.current() == AssistantState.IDLE
