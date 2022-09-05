from transition_graph import RoadNetwork, TransitionGraph
from state import SystemState
from mcts import mcts
from logger import *

import pandas as pd


def main(mine_type='1', n_truck=3, t_s=0, t_end=1200.0):
    n = n_truck
    t_e = t_end  # [sec]

    road_network = RoadNetwork(mine_type)
    transition_graph = TransitionGraph(road_network.R)
    transition_graph.show_labels_table()
    transition_graph.draw()
    transition_graph.show()


    initial_nodes = [transition_graph.find_node('A', False, False) for _ in range(n)]
    s = SystemState(transition_graph, n, t_s, t_e, initial_nodes=initial_nodes)

    reward_log = {'sim_time': [], 'objective': []}

    while s.sim_time <= s.t_e:
        s.transition()
        # s.show()

        reward_log['sim_time'].append(s.sim_time)
        reward_log['objective'].append(s.get_objective())

    logger.info(f'Score = {s.get_objective():4.3} @ {s.t_e}[sec]')
    df = pd.DataFrame(reward_log)
    df.to_csv(f'random_result_{mine_type}.csv')


def mcts_main(mine_type='1', n_truck=3, t_s=0, t_end=1200.0):
    n = n_truck
    t_e = t_end  # [sec]

    road_network = RoadNetwork(mine_type)
    transition_graph = TransitionGraph(road_network.R)
    initial_nodes = [transition_graph.find_node('A', False, False) for _ in range(n)]
    s = SystemState(transition_graph, n, t_s, t_e, initial_nodes=initial_nodes)

    searcher = mcts(iterationLimit=15)

    reward_log = {'sim_time': [], 'objective': []}

    while s.sim_time <= s.t_e:
        action = searcher.search(initialState=s)
        s = s.takeAction(action)

        reward_log['sim_time'].append(s.sim_time)
        reward_log['objective'].append(s.get_objective())

    logger.info(f'Score = {s.get_objective():4.3} @ {s.t_e}[sec]')
    df = pd.DataFrame(reward_log)
    df.to_csv(f'mcts_result_{mine_type}.csv')


if __name__ == '__main__':
    mine_type = '6'
    n = 5
    t_start = 0
    t_end = 100.0 * 60.0

    main(mine_type=mine_type, n_truck=n, t_s=t_start, t_end=t_end)
    mcts_main(mine_type=mine_type, n_truck=n, t_s=t_start, t_end=t_end)
    logger.info('EOP')
