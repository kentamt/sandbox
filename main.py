from transition_graph import RoadNetwork, TransitionGraph
from state import SystemState
from logger import *

def main():
    road_network = RoadNetwork('1')
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
