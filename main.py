import time
import joblib
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
    initial_nodes = [transition_graph.find_node('A', False, False) for _ in range(n)]

    # transition_graph = joblib.load('sample_mine_2_trans_graph.joblib')
    # initial_nodes = [transition_graph.find_node('C2', False, False) for _ in range(n)]

    transition_graph.show_labels_table()
    # transition_graph.draw()
    # transition_graph.show()

    s = SystemState(transition_graph, n, t_s, t_e, initial_nodes=initial_nodes)

    reward_log = {'sim_time': [], 'objective': []}

    while s.sim_time <= s.t_e:
        s.transition()
        # s = s.takeAction()
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

    # transition_graph = joblib.load('sample_mine_2_trans_graph.joblib')
    # initial_nodes = [transition_graph.find_node('C2', False, False) for _ in range(n)]



    s = SystemState(transition_graph, n, t_s, t_e, initial_nodes=initial_nodes)

    reward_log = {'sim_time': [], 'objective': []}

    while s.sim_time <= s.t_e:
        print(s.sim_time, s.t_e)
        searcher = mcts(iterationLimit=100)
        start = time.time()
        result = searcher.search(initialState=s, needDetails=True)
        print(f'{time.time() - start}[sec]')
        action = result['action']
        expected_reward = result['expectedReward']
        # print(f'{expected_reward=}')
        s = s.takeAction(action)
        print(action)
        print(s)
        print(s.get_objective())

        reward_log['sim_time'].append(s.sim_time)
        reward_log['objective'].append(s.get_objective())

    logger.info(f'Score = {s.get_objective():4.3f} @ {s.t_e}[sec]')
    df = pd.DataFrame(reward_log)
    df.to_csv(f'mcts_result_{mine_type}.csv')


if __name__ == '__main__':
    mine_type = '6'
    n = 3
    t_start = 0
    t_end = 40.0 * 60.0

    main(mine_type=mine_type, n_truck=n, t_s=t_start, t_end=t_end)
    mcts_main(mine_type=mine_type, n_truck=n, t_s=t_start, t_end=t_end)
    logger.info('EOP')
