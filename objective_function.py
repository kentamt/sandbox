from typing import Optional

import numpy as np

from transition_graph import TransitionGraph, TransitionNode, TransitionLabel, TransitionType, LocationType
from logger import *


class ObjectiveFunction:
    """"""

    def __init__(self, transition_graph: TransitionGraph):

        self.trans_graph: TransitionGraph = transition_graph

        # generate dummy plan
        dummy_plan = self.generate_dummy_plan()
        self.mine_plan: dict[(TransitionNode, TransitionNode)] = dummy_plan

        # params
        self.xi = 0.5

    def o(self, state, t: float):
        """
        """
        o_sum_a1_a2 = 0
        for (a1, a2) in self.mine_plan.keys():
            f_a1_a2 = self.f(state, a1, a2, t)
            g_a1_a2 = self.g(a1, a2, t)
            o_sum_a1_a2 += self.e(f_a1_a2 - g_a1_a2)
            logger.debug(f'f(({a1}, {a2}), {t}) = {f_a1_a2}')
            logger.debug(f'g(({a1}, {a2}), {t}) = {g_a1_a2}')

        o_sum_a1 = 0
        for (a1, _) in self.mine_plan.keys():
            f_a1 = self.f(state, a1, None, t)
            g_a1 = self.g(a1, None, t)
            o_sum_a1 += self.e(f_a1 - g_a1)
            logger.debug(f'f(({a1}, .), {t}) = {f_a1}')
            logger.debug(f'g(({a1}, .), {t}) = {g_a1}')

        return o_sum_a1_a2 + o_sum_a1

    def f(self,
          # state: SystemState,
          state,
          a1: TransitionNode,
          a2: Optional[TransitionNode],
          t: float):
        """
        f((a1,a2),t) is the number of times the load-unload pair (a1,a2) was fulfilled.
        f((a,.),t) measures the performance at the load location 'a' at 't'

        NOTE: it is unclear what f() actually measures. It should be in the same units as g().
        NOTE: state.r is accumulated with time, so don't multiply 't'
        """

        # update load-unload pair's reward buckets
        load_unload_reward_buckets: dict = {}
        for edge in self.trans_graph.G.edges:
            n_f = edge[0]
            n_t = edge[1]
            trans_e: TransitionLabel = self.trans_graph.G.edges[(n_f, n_t)]['transition']
            if trans_e.a is not None:
                load_unload_reward_buckets[trans_e.a] = state.r[(n_f, n_t)]

        # f((a,.),t) = sum_{a2∈A} {f(a, a2, t)
        if a2 is None:
            ret = 0
            for (_a1, _a2) in self.mine_plan:
                if _a1 == a1:
                    ret += load_unload_reward_buckets[(a1.loc_name, None)]
        # f((a1, a2), t)
        else:
            ret = load_unload_reward_buckets[(a1.loc_name, a2.loc_name)]
        return ret

    def g(self,
          a1: TransitionNode,
          a2: Optional[TransitionNode],
          t: float):
        """
        g() is a monotonically increasing function of t.
        g((a1, a2), t)
        g((a,.),t) = sum_{a2∈A} {g(a, a2, t)}
        """

        # g((a,.),t) = sum_{a2∈A} {g(a, a2, t)
        if a2 is None:
            ret = 0
            for (_a1, _a2) in self.mine_plan:
                if _a1 == a1:
                    ret += self.mine_plan[(a1, _a2)] * t
        # g((a1, a2), t)
        else:
            ret = self.mine_plan[(a1, a2)] * t

        return ret

    @staticmethod
    def e(x):
        if x <= 0:
            return 1.0 / 4.0 - (x - 1.0 / 2.0) ** 2
        else:
            return np.sqrt(x + 1.0 / 4.0) - 1.0 / 2.0

    def generate_dummy_plan(self):
        dummy_plan: dict[(TransitionNode, TransitionNode)] = dict()
        dummy_value = 2000. / 60.0 / 300.0 / 100.0  # [ton/h] * [h] / [ton]

        activity_node_list = []
        for node in self.trans_graph.G.nodes:
            if self.is_activity_node(node):
                activity_node_list.append(node)

        for node_fr in activity_node_list:
            for node_to in activity_node_list:
                if node_fr is not node_to:
                    node_fr: TransitionNode
                    node_to: TransitionNode

                    if self.is_activity_node(node_fr) and node_fr.loc_type == LocationType.ORE_LOAD:
                        if self.is_activity_node(node_to) and node_to.loc_type == LocationType.CRUSHER:
                            edge = (node_fr, node_to)
                            dummy_plan[edge] = dummy_value

        return dummy_plan

    def is_activity_node(self, node: TransitionNode):
        """
        We define
        - a node which is loading location and is_loaded, and
        - a node which is dumping location and not is_loaded
        as an activity node
        """
        if node.loc_type == LocationType.ORE_LOAD:
            if node.is_loaded and not node.star:
                return True
            else:
                return False

        elif node.loc_type == LocationType.CRUSHER:
            if not node.is_loaded and not node.star:
                return True
            else:
                return False

        else:
            return False

        # if node.loc_type == loc_type and not node.star and not node.is_loaded:
        #     return True
        # else:
        #     return False

    def update(self, t):
        return 0 * self.xi * t

    def get_val(self):
        return 0

