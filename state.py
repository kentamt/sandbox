from typing import Optional
import random
random.seed(1)
import numpy as np
from transition_graph import RoadNetwork, TransitionGraph, TransitionNode, TransitionLabel

from logger import *

class ObjectiveFunction():
    """"""

    def __init__(self, discount_factor):
        """Constructor for ObjectiveFunction"""
        self.xi = discount_factor
        self.g = GoalFunction()

    def update(self, t):
        return 0 * self.xi * t

    def get_val(self):
        return 0

    @staticmethod
    def e(x):
        if x <= 0:
            return 1.0/4.0 - (x - 1.0/2.0)**2
        else:
            return np.sqrt(x + 1.0 / 4.0) - 1.0/2.0

class GoalFunction():
    """
    return
    """

    def __init__(self, ):
        """Constructor for GoalFunction"""
        pass

    def get_val(self, a1, a2, t):

        if a2 is None:
            return 5 * t  # DEBUG: set value from the upper solution
        else:
            return 1 * t


class SystemState:
    """"""

    def __init__(self, transition_graph, num_vehicles, num_locations, num_pairs):
        """Constructor for """
        self.trans_graph: TransitionGraph = transition_graph
        self.d: list[Optional[TransitionNode]]= [None] * num_vehicles  # a vertex id that the i-th truck is heading to.
        self.td = [0] * num_vehicles  # an estimated time of arrival at the next node
        self.u = [0] * num_locations # a time that the i-th truck will finish/finished its task
        self.r = [0] * num_pairs  # total reward the i-th node has got
        self.o1 = 0 # objective value
        self.o2 = 0 # objective value
        self.t = 0  # the last point in time when the objective function was updated

        self.o = ObjectiveFunction(0.5)

    def __str__(self):
        ret = f"t={self.t},\t"
        ret += "d=["
        for e in self.d:
            ret += f"{e} "
        ret = ret[:-1]
        ret += ']\t, '

        ret += "td=["
        for e in self.td:
            ret += f"{e} "
        ret = ret[:-1]
        ret += '],\t'

        ret += "u=["
        for e in self.u:
            ret += f"{e} "
        ret = ret[:-1]
        ret += '],\t'

        ret += "r=["
        for e in self.r:
            ret += f"{e} "
        ret = ret[:-1]
        ret += '],\t'

        ret += f"o1={self.o1},\t"
        ret += f"o2={self.o2},\t"


        return ret

    def init(self, t: float):
        """
        give trucks initial positions

        state doesn't include the current position of each truck.
        It only contains the destination.
        We give their origins as destination with estimated arrival time = 0

        """
        # DEBUG:
        # All vehicle start from 'A' as empty
        import random
        random.seed(1)
        dumping_locs = self.trans_graph.get_loc_names(loc_type='dumping')
        dumping_nodes = [self.trans_graph.find_node(loc, False, False) for loc in dumping_locs]

        for idx, _ in enumerate(self.d):
            org_node = random.choice(dumping_nodes)
            self.d[idx] = org_node
            self.td[idx] = 0  # all trucks arrive at the org_nodes at 0:00


    def transition(self, dt: float):
        """"""

        # Find the fastest truck that arrives at its destination
        i = np.argmin(self.td)

        # The simulation time equals the time when the truck arrives.
        self.t = self.td[i]

        # Find the next action
        neighbors: list[TransitionNode] = list(self.trans_graph.G.neighbors(self.d[i])) # FIXME: it should be wrapped.
        node_to: TransitionNode = random.choice(neighbors)
        node_fr: TransitionNode = self.d[i]
        trans_e: TransitionLabel = self.trans_graph.G.edges[(node_fr, node_to)]['transition']
        print(trans_e)

        self.d[i] = node_to
        f_e = trans_e.f
        u_ke = 0
        self.td[i] = max(self.td[i], u_ke + f_e)
        self.u[i] = self.td[i]
        r_e = 1.0
        # self.r[i] = self.r[i] + r_e
        self.o1 = self.o.update(self.t)
        self.o2 = self.o2 + self.o.xi ** 1.0 / dt




    def evaluate(self):
        """"""
        pass

    def show(self):
        """"""
        logger.info(self)

def main():
    road_network = RoadNetwork('1')
    transition_graph = TransitionGraph(road_network.R)

    n = 3
    m = 5
    p = 2  # the number of load-unload pair
    s = SystemState(transition_graph, n, m, p)

    t = 0  # [sec]
    dt = 5.0  # [sec]

    s.init(t)
    for _ in range(10):
        s.transition(dt)
        s.show()
        t += dt

    s.evaluate()

if __name__ == '__main__':
    main()
    logger.info('EOP')