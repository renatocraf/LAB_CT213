import numpy as np
from math import inf, fabs
from utils import *


def random_policy(grid_world):
    """
    Creates a random policy for a grid world.

    :param grid_world: the grid world.
    :type grid_world: GridWorld.
    :return: random policy.
    :rtype: tridimensional NumPy array.
    """
    dimensions = grid_world.dimensions
    policy = (1.0 / NUM_ACTIONS) * np.ones((dimensions[0], dimensions[1], NUM_ACTIONS))
    return policy


def greedy_policy(grid_world, value, epsilon=1.0e-3):
    """
    Computes a greedy policy considering a value function for a grid world. If there are more than
    one optimal action for a given state, then the optimal action is chosen at random.


    :param grid_world: the grid world.
    :type grid_world: GridWorld.
    :param value: the value function.
    :type value: bidimensional NumPy array.
    :param epsilon: tolerance used to consider that more than one action is optimal.
    :type epsilon: float.
    :return: greedy policy.
    :rtype: tridimensional NumPy array.
    """
    dimensions = grid_world.dimensions
    policy = np.zeros((dimensions[0], dimensions[1], NUM_ACTIONS))
    for i in range(dimensions[0]):
        for j in range(dimensions[1]):
            current_state = (i, j)
            if not grid_world.is_cell_valid(current_state):
                # Assuming random action if the cell is an obstacle
                policy[i, j] = (1.0 / NUM_ACTIONS) * np.ones(NUM_ACTIONS)
                continue
            max_value = -inf
            action_value = np.zeros(NUM_ACTIONS)  # Creating a temporary q(s, a)
            for action in range(NUM_ACTIONS):
                r = grid_world.reward(current_state, action)
                action_value[action] = r
                for next_state in grid_world.get_valid_sucessors((i, j), action):
                    transition_prob = grid_world.transition_probability(current_state, action, next_state)
                    action_value[action] += grid_world.gamma * transition_prob * value[next_state[0], next_state[1]]
                if action_value[action] > max_value:
                    max_value = action_value[action]
            # This post-processing is necessary since we may have more than one optimal action
            num_actions = 0
            for action in range(NUM_ACTIONS):
                if fabs(max_value - action_value[action]) < epsilon:
                    policy[i, j, action] = 1.0
                    num_actions += 1
            for action in range(NUM_ACTIONS):
                policy[i, j, action] /= num_actions
    return policy


def policy_evaluation(grid_world, initial_value, policy, num_iterations=10000, epsilon=1.0e-5):
    """
    Executes policy evaluation for a policy executed on a grid world.

    :param grid_world: the grid world.
    :type grid_world: GridWorld.
    :param initial_value: initial value function used to bootstrap the algorithm.
    :type initial_value: bidimensional NumPy array.
    :param policy: policy to be evaluated.
    :type policy: tridimensional NumPy array.
    :param num_iterations: maximum number of iterations used in policy evaluation.
    :type num_iterations: int.
    :param epsilon: tolerance used in stopping criterion.
    :type epsilon: float.
    :return: value function of the given policy.
    :rtype: bidimensional NumPy array.
    """
    dimensions = grid_world.dimensions
    value = np.copy(initial_value)

    # Todo: implement policy evaluation.

    it = 0
    diferencas = np.zeros([dimensions[0], dimensions[1]])
    diff = inf

    while it < num_iterations and diff >= epsilon:
        prev_value = np.copy(value)
        for i in range(dimensions[0]):
            for j in range(dimensions[1]):
                current_state = (i,j)
                if not grid_world.is_cell_valid(current_state):
                    continue
                value[i][j]=0
                for action in range(NUM_ACTIONS):
                    r = grid_world.reward(current_state, action)
                    pi = policy[i, j, action]
                    value[i][j] += pi * r
                    for next_state in grid_world.get_valid_sucessors(current_state,action):
                        transition_prob = grid_world.transition_probability(current_state, action, next_state)
                        value[i][j] += grid_world.gamma * pi * transition_prob * prev_value[next_state[0], next_state[1]]
                diferencas[i][j] = value[i][j] - prev_value[i][j]

        diff = np.max(np.absolute(diferencas))

        it += 1
    return value


def value_iteration(grid_world, initial_value, num_iterations=10000, epsilon=1.0e-5):
    """
    Executes value iteration for a grid world.

    :param grid_world: the grid world.
    :type grid_world: GridWorld.
    :param initial_value: initial value function used to bootstrap the algorithm.
    :type initial_value: bidimensional NumPy array.
    :param num_iterations: maximum number of iterations used in policy evaluation.
    :type num_iterations: int.
    :param epsilon: tolerance used in stopping criterion.
    :type epsilon: float.
    :return value: optimal value function.
    :rtype value: bidimensional NumPy array.
    """
    dimensions = grid_world.dimensions
    value = np.copy(initial_value)
    # Todo: implement value iteration.
    it = 0
    diferencas = np.zeros([dimensions[0], dimensions[1]])
    diff = inf

    while it < num_iterations and diff >= epsilon:
        prev_value = np.copy(value)
        for i in range(dimensions[0]):
            for j in range(dimensions[1]):
                current_state = (i, j)
                if not grid_world.is_cell_valid(current_state):
                    # Assuming random action if the cell is an obstacle
                    # policy[i, j] = (1.0 / NUM_ACTIONS) * np.ones(NUM_ACTIONS)
                    continue
                max_valor = -inf

                for action in range(NUM_ACTIONS):
                    r = grid_world.reward(current_state, action)
                    value[i][j] = r
                    for next_state in grid_world.get_valid_sucessors(current_state, action):
                        transition_prob = grid_world.transition_probability(current_state, action, next_state)
                        value[i][j] += grid_world.gamma * transition_prob * prev_value[next_state[0], next_state[1]]
                    if(value[i][j] > max_valor):
                        max_valor = value[i][j]
                diferencas[i][j] = fabs(prev_value[i][j] - max_valor)
                value[i][j] = max_valor
        diff = np.max(np.abs(diferencas))
        it += 1

    return value


def policy_iteration(grid_world, initial_value, initial_policy, evaluations_per_policy=3, num_iterations=10000,
                     epsilon=1.0e-5):
    """
    Executes policy iteration for a grid world.

    :param grid_world: the grid world.
    :type grid_world: GridWorld.
    :param initial_value: initial value function used to bootstrap the algorithm.
    :type initial_value: bidimensional NumPy array.
    :param initial_policy: initial policy used to bootstrap the algorithm.
    :type initial_policy: tridimensional NumPy array.
    :param evaluations_per_policy: number of policy evaluations per policy iteration.
    :type evaluations_per_policy: int.
    :param num_iterations: maximum number of iterations used in policy evaluation.
    :type num_iterations: int.
    :param epsilon: tolerance used in stopping criterion.
    :type epsilon: float.
    :return value: value function of the optimal policy.
    :rtype value: bidimensional NumPy array.
    :return policy: optimal policy.
    :rtype policy: tridimensional NumPy array.
    """
    value = np.copy(initial_value)
    policy = np.copy(initial_policy)
    dimensions = grid_world.dimensions
    # Todo: implement policy iteration.
    it = 0
    diferencas = np.zeros([dimensions[0], dimensions[1]])
    diff = inf

    while it < num_iterations and diff >= epsilon:
        prev_value = np.copy(value)
        for k in range(evaluations_per_policy):
            for i in range(dimensions[0]):
                for j in range(dimensions[1]):
                    current_state = (i, j)
                    if not grid_world.is_cell_valid(current_state):
                        # Assuming random action if the cell is an obstacle
                        # policy[i, j] = (1.0 / NUM_ACTIONS) * np.ones(NUM_ACTIONS)
                        continue
                    value[i][j] = 0
                    for action in range(NUM_ACTIONS):
                        r = grid_world.reward(current_state, action)
                        pi = policy[i, j, action]
                        value[i][j] += pi * r
                        for next_state in grid_world.get_valid_sucessors(current_state, action):
                            transition_prob = grid_world.transition_probability(current_state, action, next_state)
                            value[i][j] += grid_world.gamma * pi * transition_prob * prev_value[next_state[0], next_state[1]]
                    diferencas[i][j] = value[i][j] - prev_value[i][j]
            diff = np.max(np.abs(diferencas))
        if (diff > epsilon):
            policy = greedy_policy(grid_world, value, epsilon)
        it += 1

    return value, policy

