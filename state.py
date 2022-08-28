from typing import Optional
import random

random.seed(1)
import numpy as np
from transition_graph import RoadNetwork, TransitionGraph, TransitionNode, TransitionLabel, TransitionType, LocationType

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
            return 1.0 / 4.0 - (x - 1.0 / 2.0) ** 2
        else:
            return np.sqrt(x + 1.0 / 4.0) - 1.0 / 2.0


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

    def __init__(self, transition_graph, num_vehicles):
        """Constructor for SystemStete"""

        self.trans_graph: TransitionGraph = transition_graph
        self.o = ObjectiveFunction(0.5)

        # prams
        # TODO: is it determined by material types?
        self._r_for_loading = 1.0
        self._r_for_dumping = 1.0

        # state
        self.d: list[Optional[TransitionNode]] = [None] * num_vehicles   # vertex id that the i-th truck is heading to
        self.td: list[float] = [0] * num_vehicles                        # estimated time of arrival at the next node
        self.u: dict[TransitionNode] = self.__init_u_ke()                # FIXME: is it good to have it as a dict?
        self.r: dict[(TransitionNode, TransitionNode)] = self.__init_r()  # total reward the i-th node has got
        self.o1: float = 0  # objective value
        self.o2: float = 0  # objective value
        self.t: float = 0  # the last point in time when the objective function was updated






    def __init_u_ke(self):
        ret = dict()
        for node in self.trans_graph.G.nodes:
            ret[node] = 0.0

        return ret

    def __init_r(self):
        ret = dict()
        for edge in self.trans_graph.G.edges:
            node_fr: TransitionNode = edge[0]
            node_to: TransitionNode = edge[1]
            if node_fr.star:
                ret[edge] = self._r_for_loading
            elif node_to.star:
                ret[edge] = self._r_for_dumping
            else:
                ret[edge] = 0.0
        return ret

    def __str__(self):
        ret = f"t={self.t},\n"
        ret += "d=["
        for e in self.d:
            ret += f"{e} "
        ret = ret[:-1]
        ret += '], \n'

        ret += "td=["
        for e in self.td:
            ret += f"{e} "
        ret = ret[:-1]
        ret += '],\n'

        ret += "u=["
        for e in self.u:
            if self.u[e] != 0:
                ret += f"{e}: {self.u[e]} "
        ret = ret[:-1]
        ret += '],\n'

        ret += "r=["
        for e in self.r:
            if self.r[e] != 0:
                ret += f"{e}: {self.r[e]} "
        ret = ret[:-1]
        ret += '],\n'

        ret += f"o1={self.o1},\n"
        ret += f"o2={self.o2},\n"

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
        logger.debug(f'Vehicle ID = {i}')

        # The simulation time equals the time when the truck arrives.
        self.t = self.td[i]

        # Find the next action
        neighbors: list[TransitionNode] = list(self.trans_graph.G.neighbors(self.d[i]))  # FIXME: it should be wrapped.
        node_to: TransitionNode = random.choice(neighbors)
        node_fr: TransitionNode = self.d[i]
        trans_e: TransitionLabel = self.trans_graph.G.edges[(node_fr, node_to)]['transition']
        logger.debug(trans_e)
        l_e = trans_e.l
        f_e = trans_e.f
        r_e = trans_e.r

        self.d[i] = node_to
        self.td[i] = max(self.td[i] + l_e, self.u[node_fr] + f_e)
        self.u[node_fr] = self.td[i]         # FIXME: check if node_fr is correct
        self.r[(node_fr, node_to)] += r_e    # FIXME: check if node_fr is correct
        self.o1 = self.o.update(self.t)
        self.o2 = self.o2 + self.o.xi ** 1.0 / dt

    def evaluate(self):
        """"""
        pass

    def show(self):
        """"""
        logger.debug(self)


def main():
    road_network = RoadNetwork('1')
    transition_graph = TransitionGraph(road_network.R)

    n = 3
    # m = 5
    # p = 2  # the number of load-unload pair
    s = SystemState(transition_graph, n)

    t = 0  # [sec]
    dt = 5.0  # [sec]

    s.init(t)
    for _ in range(20):
        s.transition(dt)
        s.show()
        t += dt

    s.evaluate()


if __name__ == '__main__':
    main()
    logger.info('EOP')
