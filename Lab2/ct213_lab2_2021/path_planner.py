from grid import Node, NodeGrid
from math import inf
import heapq

class PathPlanner(object):
    """
    Represents a path planner, which may use Dijkstra, Greedy Search or A* to plan a path.
    """
    def __init__(self, cost_map):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)

    @staticmethod
    def construct_path(goal_node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :type goal_node: Node.
        :return: the path as a sequence of (x, y) positions: [(x1,y1),(x2,y2),(x3,y3),...,(xn,yn)].
        :rtype: list of tuples.
        """
        node = goal_node
        # Since we are going from the goal node to the start node following the parents, we
        # are transversing the path in reverse
        reversed_path = []
        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent
        return reversed_path[::-1]  # This syntax creates the reverse list

    def dijkstra(self, start_position, goal_position):
        """
        Plans a path using the Dijkstra algorithm.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the Dijkstra algorithm
		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        self.node_grid.reset()
        #pegando os nos
        node_inicio = self.node_grid.get_node(start_position[0], start_position[1])
        node_final = self.node_grid.get_node(goal_position[0], goal_position[1])
        #prioridade do no inicial eh 0
        node_inicio.f = 0
        #criando fila de prioridade
        pq=[]
        #inserindo no inicial na fila de prioridade
        heapq.heappush(pq, (node_inicio.f, node_inicio))

        while pq != []:
            #tira o menor no
            prior, node = heapq.heappop(pq)
            #visita o no
            node.closed = True
            # se for o no final, termina
            if node == node_final:
                break
            #pega os sucessores
            sucessores = self.node_grid.get_successors(node.i,node.j)
            #calcula a prioridade de cada um e insere na fila de prioridades
            tupla_node = (node.i, node.j)
            for tupla_filho in sucessores:
                filho = self.node_grid.get_node(tupla_filho[0], tupla_filho[1])
                if not filho.closed:
                    # custo do inicio ate o filho
                    custo = node.f+self.cost_map.get_edge_cost(tupla_node, tupla_filho)
                    if (filho.f > custo):
                        # se f do filho for maior que o (f do pai + custo), troca o f e o parent
                        filho.f = custo
                        filho.parent = node
                        #insere filho na fila de prioridade
                        heapq.heappush(pq, (filho.f, filho))

        return self.construct_path(node_final), node_final.f

    def greedy(self, start_position, goal_position):
        """
        Plans a path using greedy search.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the Greedy Search algorithm
		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        self.node_grid.reset()
        # pegando os nos
        node_inicio = self.node_grid.get_node(start_position[0], start_position[1])
        node_final = self.node_grid.get_node(goal_position[0], goal_position[1])
        # f -> prioridade
        # g -> custo

        #setando prioridade e custo inicial
        node_inicio.f = 0
        node_inicio.g = node_inicio.distance_to(node_final.i,node_final.j)

        # criando fila de prioridade
        pq = []
        # inserindo noh inicial na fila de prioridade
        heapq.heappush(pq, (node_inicio.f, node_inicio))
        node_inicio.closed= True

        while pq!= []:
            # tira o menor no
            prior, node = heapq.heappop(pq)
            tupla_node = (node.i, node.j)
            # visita o no
            #node.closed = True
            #pega os sucessores
            sucessores = self.node_grid.get_successors(node.i, node.j)
            #calcula a prioridade de cada um e insere na fila de prioridades
            for tupla_filho in sucessores:
                filho = self.node_grid.get_node(tupla_filho[0], tupla_filho[1])
                if not filho.closed:
                    #setando pai
                    filho.parent = node
                    #setando prioridade
                    filho.g = filho.distance_to(node_final.i, node_final.j)
                    #setando custo
                    filho.f = node.f + self.cost_map.get_edge_cost(tupla_node,tupla_filho)
                    if filho == node_final:
                        return self.construct_path(node_final), node_final.f
                    # insere filho na fila de prioridade
                    heapq.heappush(pq, (filho.g, filho))
                    #visita o filho
                    filho.closed = True

        return self.construct_path(node_final), node_final.f

    def a_star(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the A* algorithm
		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        self.node_grid.reset()
        # pegando os nos
        node_inicio = self.node_grid.get_node(start_position[0], start_position[1])
        node_final = self.node_grid.get_node(goal_position[0], goal_position[1])
        # f -> prioridade
        # g -> custo

        # setando prioridade e custo inicial
        node_inicio.f = node_inicio.distance_to(node_final.i, node_final.j)
        node_inicio.g = 0

        # criando fila de prioridade
        pq = []
        # inserindo noh inicial na fila de prioridade
        heapq.heappush(pq, (node_inicio.f, node_inicio))

        while pq !=[]:
            # tira o menor no
            prior, node = heapq.heappop(pq)
            if node == node_final:
                return self.construct_path(node_final), node_final.f
            tupla_node = (node.i, node.j)
            # visita o no
            node.closed = True
            # pega os sucessores
            sucessores = self.node_grid.get_successors(node.i, node.j)
            # calcula a prioridade de cada um e insere na fila de prioridades
            for tupla_filho in sucessores:
                filho = self.node_grid.get_node(tupla_filho[0], tupla_filho[1])
                if not filho.closed:
                    newG = node.g + self.cost_map.get_edge_cost(tupla_node, tupla_filho)
                    dist = filho.distance_to(node_final.i,node_final.j)
                    if filho.f > newG + dist:
                        filho.g = newG
                        filho.f = filho.g + dist
                        # setando pai
                        filho.parent = node
                        # insere filho na fila de prioridade
                        heapq.heappush(pq, (filho.f, filho))

