import numpy as np
import random
from math import inf


class Particle:
    """
    Represents a particle of the Particle Swarm Optimization algorithm.
    """
    def __init__(self, lower_bound, upper_bound):
        """
        Creates a particle of the Particle Swarm Optimization algorithm.

        :param lower_bound: lower bound of the particle position.
        :type lower_bound: numpy array.
        :param upper_bound: upper bound of the particle position.
        :type upper_bound: numpy array.
        """
        # Todo: implement
        #pass  # Remove this line
        # lb e up sao np.array
        self.x = np.random.uniform(lower_bound, upper_bound)
        delta = upper_bound - lower_bound
        self.v = np.random.uniform(-delta, delta)
        self.value = 0
        self.best_it = self.x
        self.best_it_value = -inf


class ParticleSwarmOptimization:
    """
    Represents the Particle Swarm Optimization algorithm.
    Hyperparameters:
        inertia_weight: inertia weight.
        cognitive_parameter: cognitive parameter.
        social_parameter: social parameter.

    :param hyperparams: hyperparameters used by Particle Swarm Optimization.
    :type hyperparams: Params.
    :param lower_bound: lower bound of particle position.
    :type lower_bound: numpy array.
    :param upper_bound: upper bound of particle position.
    :type upper_bound: numpy array.
    """
    def __init__(self, hyperparams, lower_bound, upper_bound):
        # Todo: implement
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.delta = upper_bound - lower_bound
        self.hyperparams = hyperparams

        # gerando as particulas
        self.initialize_particles()

        self.best_global = None
        self.best_global_value = - inf

        self.position = 0

    def initialize_particles(self):
        """
        Create vector of particles and initialize.

        :return: void
        """
        self.particulas = []
        for i in range(0, self.hyperparams.num_particles):
            self.particulas.append(Particle(self.lower_bound, self.upper_bound))

    def get_best_position(self):
        """
        Obtains the best position so far found by the algorithm.

        :return: the best position.
        :rtype: numpy array.
        """
        # Todo: implement
        return self.best_global

    def get_best_value(self):
        """
        Obtains the value of the best position so far found by the algorithm.

        :return: value of the best position.
        :rtype: float.
        """
        # Todo: implement
        return self.best_global_value

    def get_position_to_evaluate(self):
        """
        Obtains a new position to evaluate.

        :return: position to evaluate.
        :rtype: numpy array.
        """
        # Todo: implement
        return self.particulas[self.position].x

    def advance_generation(self):
        """
        Advances the generation of particles. Auxiliary method to be used by notify_evaluation().
        """
        # Todo: implement

        w = self.hyperparams.inertia_weight
        phip = self.hyperparams.cognitive_parameter
        phig = self.hyperparams.social_parameter

        particle = self.particulas[self.position]

        if particle.value > self.best_global_value:
            self.best_global = particle.x
            self.best_global_value = particle.value
        if particle.value > particle.best_it_value:
            particle.best_it = particle.x
            particle.best_it_value = particle.value

        rp = random.uniform(0, 1)
        rg = random.uniform(0, 1)

        particle.v = w * particle.v + phip * rp * (particle.best_it - particle.x) + phig * rg * (self.best_global - particle.x)
        #ajustando limites de v
        particle.v = np.minimum(np.maximum(-self.delta, particle.v), self.delta)

        particle.x = particle.x + particle.v
        #ajustando limites de x
        particle.x = np.minimum(np.maximum(self.lower_bound, particle.x), self.upper_bound)

        # gerando nova posição
        self.position = (self.position + 1) % self.hyperparams.num_particles


    def notify_evaluation(self, value):
        """
        Notifies the algorithm that a particle position evaluation was completed.

        :param value: quality of the particle position.
        :type value: float.
        """
        # Todo: implement
        self.particulas[self.position].value = value
        self.advance_generation()

