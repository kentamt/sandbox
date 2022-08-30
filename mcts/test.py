from network import Network
# from heuristic import greedy, two_opt
from mcts import RandomMCTS, GreedyMCTS
from plot import plot_path
from matplotlib import pyplot as plt
import time


def test(num_of_node, side_length=100, plot=False):
    network = Network(num_of_node, side_length)
    edges_set = []
    cost_set = []
    run_time_set = []

    ### mcts 1 - random
    start = time.time()
    random_mcts = RandomMCTS(network)
    edges, cost = random_mcts.run(50, 100, 1000)  # run takes (number to expand, number to simulate, and constant C) as input
    run_time = time.time() - start

    edges_set.append(edges)
    cost_set.append(cost)
    run_time_set.append(run_time)
    print ("random mcts has cost of {:.2f} using {:.4f}s".format(cost, run_time))

    ### mcts 2 - greedy
    start = time.time()
    greedy_mcts = GreedyMCTS(network, 0.2)
    edges, cost = greedy_mcts.run(50, 100, 100)  # run takes (number to expand, number to simulate,
    ## and constant C) as input
    run_time = time.time() - start

    edges_set.append(edges)
    cost_set.append(cost)
    run_time_set.append(run_time)
    print ("greedy mcts has cost of {:.2f} using {:.4f}s".format(cost, run_time))

    if plot == True:
        fig, axs = plt.subplots(nrows=1, ncols=2, figsize=(9, 4))
        # fig.suptitle("Path Found by Different Models (num_of_node={:d}, side_length={:d})".format(num_of_node, side_length))
        # model_names = ['greedy heuristic', '2-opt heuristic', 'random mcts', 'greedy mcts']
        model_names = ['random mcts', 'greedy mcts']
        for i in range(2):
            plot_path(axs[i], model_names[i], cost_set[i], run_time_set[i],
                      network.graph.nodes, edges_set[i], network.positions)
        plt.show()


if __name__ == '__main__':
    test(10, plot=True)
