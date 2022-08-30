from typing import Optional
import random

import matplotlib.pyplot as plt

random.seed(1)

import numpy as np

from transition_graph import RoadNetwork, TransitionGraph, TransitionNode, TransitionLabel, TransitionType, LocationType
from objective_function import ObjectiveFunction
from logger import *


class SystemState:
    """"""

    def __init__(self, transition_graph, num_vehicles, t_s, t_e):
        """Constructor for SystemState"""

        # simulation time
        self.t_s = t_s  # [sec]
        self.t_e = t_e  # [sec]
        self.sim_time = t_s  # [sec]

        # objects
        self.trans_graph: TransitionGraph = transition_graph
        self.o = ObjectiveFunction(transition_graph)

        # prams
        # TODO: is it determined by material types?
        self._r_for_loading = 1.0
        self._r_for_dumping = 1.0

        # state
        self.d: list[Optional[TransitionNode]] = [None] * num_vehicles  # vertex id that the i-th truck is heading to
        self.td: list[float] = [0] * num_vehicles  # estimated time of arrival at the next node
        self.u: dict[TransitionNode] = self.__init_u_ke()  # FIXME: is it good to have it as a dict?
        self.r: dict[(TransitionNode, TransitionNode)] = self.__init_r()  # total reward the i-th node has got
        self.o1: float = 0  # objective value
        self.o2: float = 0  # objective value
        self.t: float = t_s  # the last point in time when the objective function was updated

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
                # ret[edge] = self._r_for_loading
                ret[edge] = 0.0
            elif node_to.star:
                # ret[edge] = self._r_for_dumping
                ret[edge] = 0.0
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
        # ret = ret[:-1]
        ret += '],\n'

        ret += f"o1={self.o1},\n"
        ret += f"o2={self.o2},\n"

        return ret

    def get_objective(self):
        return self.o2

    def init(self):
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
            self.td[idx] = self.t_s  # [sec]

    def get_reward_bucket(self,
                          a1: str,  # TransitionNode,
                          a2: Optional[str]):
        """
        a1: loading node
        a2: unloading node
        """
        reward_bucket = None
        for edge in self.r.keys():
            node_fr: TransitionNode = edge[0]
            node_to: TransitionNode = edge[1]

            if a2 is None:
                if node_fr.loaded_loc is not None:
                    if node_fr.loaded_loc.loc_name == a1 and node_to.loc_name == a1 and node_fr.loc_name == a1 and node_fr.star:
                        reward_bucket = self.r[edge]
            else:
                if node_fr.loaded_loc is not None:
                    if node_fr.loaded_loc.loc_name == a1 and node_to.loc_name == a2 and node_fr.loc_name == a2 and node_fr.star:
                        reward_bucket = self.r[edge]

        return reward_bucket

    def transition(self):
        """"""
        if self.sim_time > self.t_e:
            return

        # Find the fastest truck that arrives at its destination
        i = np.argmin(self.td)

        # The simulation time equals the time when the truck arrives.
        new_t = self.td[i]
        dt = new_t - self.t
        self.t = new_t
        self.sim_time += dt  # [sec]  # FIXME: to organise these lines

        # Find the next action
        # FIXME: at the moment, the next node is randomly selected.
        neighbors: list[TransitionNode] = self.trans_graph.get_neighbors(self.d[i])
        node_to: TransitionNode = random.choice(neighbors)
        node_fr: TransitionNode = self.d[i]
        trans_e: TransitionLabel = self.trans_graph.G.edges[(node_fr, node_to)]['transition']
        l_e = trans_e.l
        f_e = trans_e.f
        r_e = trans_e.r

        self.d[i] = node_to
        self.td[i] = max(self.td[i] + l_e, self.u[node_fr] + f_e)
        self.u[node_fr] = self.td[i]  # TODO: check if node_fr is correct
        self.r[(node_fr, node_to)] += r_e  # TODO: check if node_fr is correct

        new_o1 = self.o.o(self, self.t)
        if dt != 0:
            self.o2 = self.o2 + self.o.xi ** (1.0 / dt) * (new_o1 - self.o1)
        else:
            pass  # TODO:  check if this is expected behavior
        self.o1 = new_o1

    def evaluate(self):
        """"""
        pass

    def show(self):
        """"""
        logger.debug(self)

        # fig, axes = self.trans_graph.draw()
        # node: TransitionNode
        # x_list, y_list = [], []
        # for x, y, nx, ny, u in zip(self.vehicle_x, self.vehicle_y,
        #                            self.vehicle_nx, self.vehicle_ny,
        #                            self.vehicle_u):
        #     if x is not None:
        #         # x_list.append(x)
        #         # y_list.append(y)
        #         axes[0].scatter(x, y, s=100, zorder=3)
        # self.trans_graph.show()
        #


def main():
    road_network = RoadNetwork('5')
    transition_graph = TransitionGraph(road_network.R)
    transition_graph.show_labels_table()
    transition_graph.draw()
    transition_graph.show()

    n = 3
    t_s = 0  # [sec]
    t_e = 200.0 * 60.0  # [sec]
    s = SystemState(transition_graph, n, t_s, t_e)

    s.init()
    while True:
        s.transition()
        # s.show()
        if s.sim_time > s.t_e:
            break

    logger.info(f'Score = {s.get_objective():4.3}')

if __name__ == '__main__':
    main()
    logger.info('EOP')
