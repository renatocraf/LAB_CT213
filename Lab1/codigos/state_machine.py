import random
import math
from constants import *

class FiniteStateMachine(object):
    """
    A finite state machine.
    """
    def __init__(self, state):
        self.state = state

    def change_state(self, new_state):
        self.state = new_state

    def update(self, agent):
        self.state.check_transition(agent, self)
        self.state.execute(agent)

class State(object):
    """
    Abstract state class.
    """
    def __init__(self, state_name):
        """
        Creates a state.

        :param state_name: the name of the state.
        :type state_name: str
        """
        self.state_name = state_name

    def check_transition(self, agent, fsm):
        """
        Checks conditions and execute a state transition if needed.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent):
        """
        Executes the state logic.

        :param agent: the agent where this state is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

class MoveForwardState(State):
    def __init__(self):
        super().__init__("MoveForward")
        # Todo: add initialization code
        self.tempo = 0.0

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition
        if self.tempo > MOVE_FORWARD_TIME:
            state_machine.change_state(MoveInSpiralState())
        if agent.get_bumper_state():
            agent.set_bumper_state(False)
            state_machine.change_state(GoBackState())
        self.tempo += SAMPLE_TIME

    def execute(self, agent):
        # Todo: add execution logic
        agent.set_velocity(FORWARD_SPEED, 0)

class MoveInSpiralState(State):
    def __init__(self):
        super().__init__("MoveInSpiral")
        # Todo: add initialization code
        self.tempo = 0.0
        self.raio = INITIAL_RADIUS_SPIRAL
    
    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition
        if self.tempo > MOVE_IN_SPIRAL_TIME:
            self.tempo = 0.0
            state_machine.change_state(MoveForwardState())
        if agent.get_bumper_state():
            agent.set_bumper_state(False)
            state_machine.change_state(GoBackState())
        self.tempo += SAMPLE_TIME
        self.raio = INITIAL_RADIUS_SPIRAL + SPIRAL_FACTOR * self.tempo

    def execute(self, agent):
        # Todo: add execution logic
        w = math.sqrt(math.pow(FORWARD_SPEED,2)+math.pow(SPIRAL_FACTOR,2))/self.raio
        agent.set_velocity(FORWARD_SPEED, w)
        #agent.move()


class GoBackState(State):
    def __init__(self):
        super().__init__("GoBack")
        # Todo: add initialization code
        self.tempo = 0.0

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition
        if self.tempo > GO_BACK_TIME:
            state_machine.change_state(RotateState())
        self.tempo += SAMPLE_TIME

    def execute(self, agent):
        # Todo: add execution logic
        agent.set_velocity(BACKWARD_SPEED, 0)



class RotateState(State):
    def __init__(self):
        super().__init__("Rotate")
        # Todo: add initialization code
        self.tempo = 0.0
        self.angulo_aleatorio = random.randint(-180, 180)
        self.tempo_aleatorio = abs(math.radians(self.angulo_aleatorio)/ANGULAR_SPEED)
        self.sinal = 1
        if self.angulo_aleatorio < 0:
            self.sinal = -1

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition
        if self.tempo > self.tempo_aleatorio:
            state_machine.change_state(MoveForwardState())
        self.tempo += SAMPLE_TIME
    
    def execute(self, agent):
        # Todo: add execution logic
        agent.set_velocity(0, self.sinal*ANGULAR_SPEED)
