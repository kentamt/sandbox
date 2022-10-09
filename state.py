import copy
from typing import Optional
import random
random.seed(1)
import numpy as np

from transition_graph import RoadNetwork, TransitionGraph, TransitionNode, TransitionLabel, TransitionType, FASTLocationType
from objective_function import ObjectiveFunction
from logger import *
import pickle

class SystemState:
    """"""

    def __init__(self, transition_graph, num_vehicles, t_s, t_e, initial_nodes=None):
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

        if initial_nodes is not None:
            if len(initial_nodes) != len(self.d):
                logger.error('length mismatch')
                raise ValueError
            node: TransitionNode
            for i, node in enumerate(initial_nodes):
                self.d[i] = node
                self.td[i] = t_s

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
        ret = f"t={self.sim_time},\n"
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
        # dt = new_t - self.t  # simulation time step
        dt = new_t - self.sim_time
        # self.t = new_t  #
        self.sim_time += dt  # [sec]  # FIXME: to organise these lines

        # Find the next action
        # FIXME: at the moment, the next node is randomly selected.
        # Take possible actions
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

        # if dt != 0:
        if self.sim_time - self.t > 200:
            new_o1 = self.o.o(self, self.sim_time)
            self.o2 = self.o2 + self.o.xi ** (1.0 / dt) * (new_o1 - self.o1)
            self.t = self.sim_time
            self.o1 = new_o1
        else:
            pass  # TODO:  check if this is expected behavior


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

    def getCurrentPlayer(self):
        """ always return 1 because it is not a 2-player game! """
        return 1

    def getPossibleActions(self):

        possible_actions = []

        # Find the fastest truck that arrives at its destination
        truck_idx = np.argmin(self.td)

        # Find possible actions
        neighbors: list[TransitionNode] = self.trans_graph.get_neighbors(self.d[truck_idx])
        for node_to in neighbors:
            node_fr: TransitionNode = self.d[truck_idx]
            trans_e: TransitionLabel = self.trans_graph.G.edges[(node_fr, node_to)]['transition']
            action = Action(truck_idx, node_fr, node_to, trans_e)
            possible_actions.append(action)

        return possible_actions

    def takeAction(self, action):
        """"""
        # new_state = copy.deepcopy(self)
        new_state = pickle.loads(pickle.dumps(self))

        i = action.idx
        node_fr = action.node_fr
        node_to = action.node_to
        trans_e = action.trans_e
        l_e = trans_e.l
        f_e = trans_e.f
        r_e = trans_e.r

        # The simulation time equals the time when the truck arrives.
        new_t = new_state.td[i]
        # dt = new_t - new_state.t
        dt = new_t - new_state.sim_time
        # new_state.t = new_t
        new_state.sim_time += dt  # [sec]

        new_state.d[i] = node_to

        new_state.td[i] = max(new_state.td[i] + l_e, new_state.u[node_fr] + f_e)
        new_state.u[node_fr] = new_state.td[i]
        new_state.r[(node_fr, node_to)] += r_e


        # if dt != 0:
        odt = new_state.sim_time - new_state.t
        if odt > 200:
            new_o1 = new_state.o.o(new_state, new_state.t)
            new_state.o2 = new_state.o2 + new_state.o.xi ** (1.0 / dt) * (new_o1 - new_state.o1)
            new_state.t = new_state.sim_time
            new_state.o1 = new_o1
        else:
            pass  # TODO:  check if this is expected behavior


        return new_state

    def isTerminal(self):
        if self.sim_time > self.t_e:
            return True
        else:
            return False

    def getReward(self):
        return self.o2

    def __eq__(self, other):
        raise NotImplementedError()


class Action:
    def __init__(self, idx, node_fr, node_to, trans_e):
        self.idx = idx
        self.node_fr = node_fr
        self.node_to = node_to
        self.trans_e = trans_e

    def __repr__(self):
        return f'{self.idx}, {self.node_fr}==>{self.node_to},{self.trans_e}'

    def __str__(self):
        return self.__repr__()

    def __eq__(self, other):
        return self.node_to.name == other.node_to.name

    def __hash__(self):
        return hash((self.idx, self.node_fr, self.node_to, self.trans_e))


def main(mine_type='1', t_end=1200.0):
    road_network = RoadNetwork(mine_type)
    transition_graph = TransitionGraph(road_network.R)
    transition_graph.show_labels_table()
    # transition_graph.draw()
    # transition_graph.show()

    n = 3
    t_s = 0  # [sec]
    t_e = t_end  # [sec]
    initial_nodes = [transition_graph.find_node('A', False, False) for _ in range(n)]
    s = SystemState(transition_graph, n, t_s, t_e, initial_nodes=initial_nodes)

    while s.sim_time <= s.t_e:
        s.transition()
        s.show()

    logger.info(f'Score = {s.get_objective():4.3} @ {s.t_e}[sec]')


if __name__ == '__main__':
    mine_type = '6'
    t_end = 100.0 * 60.0
    main(mine_type=mine_type, t_end=t_end)
    logger.info('EOP')
