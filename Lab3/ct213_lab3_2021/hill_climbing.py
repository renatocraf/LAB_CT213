from math import inf


def hill_climbing(cost_function, neighbors, theta0, epsilon, max_iterations):
    """
    Executes the Hill Climbing (HC) algorithm to minimize (optimize) a cost function.

    :param cost_function: function to be minimized.
    :type cost_function: function.
    :param neighbors: function which returns the neighbors of a given point.
    :type neighbors: list of numpy.array.
    :param theta0: initial guess.
    :type theta0: numpy.array.
    :param epsilon: used to stop the optimization if the current cost is less than epsilon.
    :type epsilon: float.
    :param max_iterations: maximum number of iterations.
    :type max_iterations: int.
    :return theta: local minimum.
    :rtype theta: numpy.array.
    :return history: history of points visited by the algorithm.
    :rtype history: list of numpy.array.
    """
    theta = theta0
    history = [theta0]
    # Todo: Implement Hill Climbing

    custoTheta = cost_function(theta)
    cont = 0

    while not (custoTheta < epsilon or cont > max_iterations):
        best = theta
        Jbest = inf
        #encontra o vizinho com menor custo
        for neighbor in neighbors(theta):
            Jneighbor = cost_function(neighbor)
            if Jneighbor < Jbest:
                best = neighbor
                Jbest = Jneighbor
        #se o custo do melhor vizinho for maior do que o custo do theta atual, finaliza
        if Jbest > custoTheta:
            return theta, history
        theta = best
        custoTheta = Jbest
        history.append(theta)
        cont += 1

    print(history)
    return theta, history
