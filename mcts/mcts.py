import numpy as np
import copy

class Node:
    """
    Node for MCTS.
    """
    def __init__(self, parent, node, cost):
        self.parent = parent
        self.node = node
        self.cost = cost
        self.score = None
        self.estimate = None
        self.policy = None
        self.num_of_visit = 1
        self.expanded = {}
        self.expandables = None

    def calculate_score(self, C=1):
        self.score = self.estimate + C * (np.log(self.parent.num_of_visit) / self.num_of_visit) ** 0.5


class MCTS:
    """
    Parent Class for MCTS
    """

    def __init__(self):
        self.root: Node = Node(None, 'root', 0)

    def select(self, node):
        if node.policy is None:
            return node
        else:
            return self.select(node.policy)

    def expand(self, node):
        new_node = node.expandables.pop()
        new_cost = copy.deepcopy(new_node)
        if node.node != 'root':
            new_cost += 0  # TODO: add cost rule

        new_node_object = Node(node, new_node, new_cost)
        node.expanded[new_node]= new_node_object
        return new_node_object

    def backpropagate(self, node):
        scores = []

        # for key, n in node.expanded.items():
        #     if node.node != 'root':
        #         scores.append()
        #
        # if node.node != 'root':



            self.backpropagate(node.parent)

    def run(self, num_of_expand, num_of_simulate):
        score = 0
        while True:
            current_node = self.select(self.root)

            if True:  # TODO: set a condition to terminate.
                break

            for i in range(min(num_of_expand, len(current_node.expandables))):
                new_node = self.expand(current_node)
                costs = []
                for j in range(num_of_simulate):
                    costs.append(self.simulate(new_node))
                new_node.estimate = sum(costs) / num_of_simulate
                new_node.calculate_score()
        return score


        return score


    def simulate(self):
        cost = 0
        return cost