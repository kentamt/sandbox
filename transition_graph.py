import sys
from enum import Enum, auto
import networkx as nx
from matplotlib import pyplot as plt
import matplotlib

matplotlib.use('TkAgg')

class LocationType(Enum):
    ORE_LOAD = auto(),
    WST_LOAD = auto(),
    CRUSHER = auto(),
    WST_DUMP = auto(),
    STOCK_PILE = auto(),
    INTSCT = auto()

class TransitionNode:
    def __init__(self, name: str, x: float, y: float, loc_type: LocationType,activity_time,
                 loc_name, state_name, star=False):
        self.name: str = name
        self.pos: tuple[float, float] = (x, y)
        self.loc_type: LocationType = loc_type
        self.activity_time = activity_time
        self.loc_name = loc_name
        self.state_name = state_name
        self.star = star

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.__str__()


def main():

    input_type = sys.argv[1]

    # Create a networkx graph object
    road_network = nx.DiGraph()

    if input_type == '1':
        data = {
            'A': (0, 0, LocationType.CRUSHER, 90),
            'B': (1.73, 1, LocationType.ORE_LOAD, 120),
            'C': (1.73, -1, LocationType.ORE_LOAD, 120),
            'D': (2. / 1.73, 0, LocationType.INTSCT, 0),
        }
    elif input_type == '2':
        data = {
            'A': (0, 0, LocationType.CRUSHER, 90),
            'B': (1.73, 1, LocationType.ORE_LOAD, 120),
            'C': (1.73, -1, LocationType.ORE_LOAD, 120),
            'D': (2. / 1.73, 0, LocationType.INTSCT, 0),
            'E': (2., 0, LocationType.INTSCT, 0),
        }
    elif input_type == '3':
        data = {
            'A': (0.0, 0, LocationType.CRUSHER, 90),
            'B': (3.0, 1, LocationType.ORE_LOAD, 120),
            'C': (3.0, -1, LocationType.ORE_LOAD, 120),
            'D': (1.5, 0, LocationType.INTSCT, 0),
            'E': (2.5, 0, LocationType.INTSCT, 0),
            'F': (2.0, 1, LocationType.INTSCT, 0),
        }

    loc_dict = {}
    for name, attr in data.items():
        # name, x, y, loc_type, activity_time, loaded, star, loc_name):
        loc = TransitionNode(name, attr[0], attr[1], attr[2], attr[3], name, None)
        loc_dict[name] = loc

    # Add edges to to the graph object
    # Each tuple represents an edge between two nodes
    if input_type == '1':
        road_network.add_edges_from([
            (loc_dict['A'], loc_dict['D']),
            (loc_dict['D'], loc_dict['A']),
            (loc_dict['B'], loc_dict['D']),
            (loc_dict['D'], loc_dict['B']),
            (loc_dict['C'], loc_dict['D']),
            (loc_dict['D'], loc_dict['C']),
        ])
    elif input_type == '2':
        road_network.add_edges_from([
            (loc_dict['A'], loc_dict['D']),
            (loc_dict['D'], loc_dict['A']),
            (loc_dict['D'], loc_dict['E']),
            (loc_dict['E'], loc_dict['D']),
            (loc_dict['B'], loc_dict['E']),
            (loc_dict['E'], loc_dict['B']),
            (loc_dict['C'], loc_dict['D']),
            (loc_dict['D'], loc_dict['C']),
        ])
    elif input_type == '3':
        road_network.add_edges_from([
            (loc_dict['A'], loc_dict['D']),
            (loc_dict['D'], loc_dict['A']),
            (loc_dict['E'], loc_dict['D']),
            (loc_dict['D'], loc_dict['E']),
            (loc_dict['F'], loc_dict['D']),
            (loc_dict['D'], loc_dict['F']),
            (loc_dict['B'], loc_dict['E']),
            (loc_dict['E'], loc_dict['B']),
            (loc_dict['F'], loc_dict['B']),
            (loc_dict['C'], loc_dict['D']),
            (loc_dict['D'], loc_dict['C']),
        ])

    for edge in road_network.edges:
        road_network.edges[edge]["EMPTY_TRAVEL_TIME"] = 40
        road_network.edges[edge]["LOADED_TRAVEL_TIME"] = 60

    pos = {}
    for name in loc_dict.keys():
        pos[loc_dict[name]] = (loc_dict[name].pos[0], loc_dict[name].pos[1])

    dump_list = [loc for loc in road_network.nodes if
                 loc.loc_type == LocationType.CRUSHER or loc.loc_type == LocationType.WST_DUMP]
    load_list = [loc for loc in road_network.nodes if
                 loc.loc_type == LocationType.ORE_LOAD or loc.loc_type == LocationType.WST_LOAD]
    int_list = [loc for loc in road_network.nodes if loc.loc_type == LocationType.INTSCT]


    # Transition graph
    # NOTE: start from the sortest paths version for the simplicity.
    transition_graph = nx.DiGraph()

    # search cycles
    dump_list = [loc for loc in road_network.nodes if
                 loc.loc_type == LocationType.CRUSHER or loc.loc_type == LocationType.WST_DUMP]
    load_list = [loc for loc in road_network.nodes if
                 loc.loc_type == LocationType.ORE_LOAD or loc.loc_type == LocationType.WST_LOAD]

    # search for cycles from dumping point to loading point
    cycle_path_list = []
    for dump in dump_list:
        for load in load_list:

            print(list(nx.all_simple_paths(road_network, dump, load)))
            # exit()


            # find outward and return trip. there can be several trips.
            outward_paths = find_trip(dump, load, road_network, paths_type='shortest', weight='EMPTY_TRAVEL_TIME')
            return_paths = find_trip(load, dump, road_network, paths_type='shortest',  weight='LOAD_TRAVEL_TIME')
            # outward_paths = find_trip(dump, load, road_network, paths_type='all_simple')
            # return_paths = find_trip(load, dump, road_network, paths_type='all_simple')

            for outward_path in outward_paths:
                cycle_path: list[TransitionNode] = []
                for return_path in return_paths:

                    # concat the outward and return trip. remove the end of the path because we will add the return trip
                    cycle_path.extend(outward_path[:-1])
                    cycle_path.extend(return_path)
                    print(f'{cycle_path=}')
                    cycle_path_list.append(cycle_path)

            print(f'{cycle_path_list=}')

        for cycle_path in cycle_path_list:
            # a cycle in a transition graph
            transition_cycle = []

            p_loc = None
            is_loaded = False
            loaded_loc = None
            for idx, loc in enumerate(cycle_path):

                # For loadings. They have three types of nodes: (1) EMPTY, (2) LOADED*_{loc.name} and (3) LOADED_{loc.name}
                if loc.loc_type == LocationType.WST_DUMP or loc.loc_type == LocationType.CRUSHER:

                    if is_loaded:
                        # name, x, y, loc_type, activity_time, loaded, star, loc_name):
                        state = "LOADED_" + loaded_loc.loc_name
                        new_loc = TransitionNode(loc.name + '_L_' + str(loaded_loc), loc.pos[0], loc.pos[1], loc.loc_type,
                                                 loc.activity_time, loc.name, state)
                        transition_cycle.append(new_loc)

                        state = "EMPTY_S_" + loaded_loc.loc_name
                        new_loc = TransitionNode(loc.name + '_E_S_' + str(loaded_loc), loc.pos[0], loc.pos[1], LocationType.INTSCT,
                                                 loc.activity_time, loc.name, state, star=True)
                        transition_cycle.append(new_loc)
                    else:
                        state = "EMPTY"
                        new_loc = TransitionNode(loc.name + '_E', loc.pos[0], loc.pos[1], loc.loc_type,
                                                 loc.activity_time, loc.name, state)
                        transition_cycle.append(new_loc)

                # For dumpings. They have three types of nodes: (1) LOADED_{loc.name}, (2) EMPTY*_{loc.name} and (3) EMPTY
                if loc.loc_type == LocationType.WST_LOAD or loc.loc_type == LocationType.ORE_LOAD:
                    is_loaded = True
                    loaded_loc = loc
                    state = "EMPTY"
                    new_loc = TransitionNode(loc.name + '_E', loc.pos[0], loc.pos[1], loc.loc_type,
                                             loc.activity_time, loc.name, state)
                    transition_cycle.append(new_loc)

                    state = 'LOADED_S_' + loaded_loc.loc_name
                    new_loc = TransitionNode(loc.name + '_L_S_' + loaded_loc.name, loc.pos[0], loc.pos[1], loc.loc_type,
                                             loc.activity_time, loc.name, state, star=True)
                    transition_cycle.append(new_loc)

                    state = 'LOADED_' + loaded_loc.loc_name
                    new_loc = TransitionNode(loc.name + '_L_' + loaded_loc.loc_name, loc.pos[0], loc.pos[1], loc.loc_type,
                                             loc.activity_time, loc.name, state)
                    transition_cycle.append(new_loc)

                # For intersections. They have two types. (1) LOADED_{loc.name} and (2) EMPTY
                if loc.loc_type == LocationType.INTSCT:
                    if is_loaded:
                        state = 'LOADED_' + loaded_loc.loc_name
                        new_loc = TransitionNode(loc.name + '_L_' + loaded_loc.loc_name, loc.pos[0], loc.pos[1], LocationType.INTSCT,
                                                 loc.activity_time, loc.name, state)
                    else:
                        state = 'EMPTY'
                        new_loc = TransitionNode(loc.name + '_E', loc.pos[0], loc.pos[1], LocationType.INTSCT,
                                                 loc.activity_time, loc.name, state)
                    transition_cycle.append(new_loc)

            print(f'{transition_cycle=}')

            # build a transition graph
            for idx, loc in enumerate(transition_cycle):
                if p_loc is None:
                    p_loc = transition_cycle[-1]
                else:
                    p_loc = transition_cycle[idx-1]
                transition_graph.add_edge(p_loc, loc)

    # Find nodes which have the same names
    del_locs = []
    visited_id = []
    for loc1 in transition_graph.nodes:
        visited_id.append(id(loc1))
        for loc2 in transition_graph.nodes:
            if id(loc2) in visited_id:
                continue

            if id(loc1) != id(loc2) and loc1.name == loc2.name:
                del_locs.append(loc1)

    # Delete and reconnect nodes
    for loc in del_locs:

        # find duplicated nodes witch have the same name in a graph
        loc_same_names = []
        for _loc in transition_graph.nodes:
            if id(_loc) != id(loc) and _loc.name == loc.name:
                loc_same_names.append(_loc)

        # delete the target node and reconnect to the duplicated node.
        for loc_same_name in loc_same_names:

            # find in-edges and reconnect
            in_arc  = list(transition_graph.in_edges(loc))[0]
            loc_from = in_arc[0]
            loc_to   = loc_same_name
            transition_graph.add_edge(loc_from, loc_to)

            # find in-edges and reconnect
            out_arc = list(transition_graph.out_edges(loc))[0]
            loc_to = out_arc[1]
            loc_from = loc_same_name
            transition_graph.add_edge(loc_from, loc_to)

            transition_graph.remove_node(loc)

    # Draw the resulting
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    nx.draw_networkx_nodes(road_network,
                           pos=pos,
                           nodelist=load_list,
                           node_size=500,
                           node_color='lightgreen',
                           edgecolors='gray',
                           ax=axes[0])
    nx.draw_networkx_nodes(road_network,
                           pos=pos,
                           nodelist=dump_list,
                           node_size=500,
                           node_color='red',
                           edgecolors='gray',
                           ax=axes[0])
    nx.draw_networkx_nodes(road_network,
                           pos=pos,
                           nodelist=int_list,
                           node_size=500,
                           node_color='white',
                           edgecolors='gray',
                           ax=axes[0])
    nx.draw_networkx_labels(road_network,
                           pos=pos,
                           font_color="black",
                           ax=axes[0])
    nx.draw_networkx_edges(road_network,
                           pos=pos,
                           width=1.0,
                           alpha=1.0,
                           edge_color="black",
                           arrowsize=10.0,
                           connectionstyle='arc3, rad = 0.1',
                           ax=axes[0])

    transition_pos = nx.spring_layout(transition_graph, seed=200)
    nx.draw_networkx_nodes(transition_graph,
                           pos=transition_pos,
                           node_color='royalblue',
                           node_size=500,
                           ax=axes[1])
    nx.draw_networkx_edges(transition_graph,
                           pos=transition_pos,
                           arrowstyle='->',
                           arrowsize=20,
                           edge_color='salmon',
                           ax=axes[1])
    nx.draw_networkx_labels(transition_graph,
                           pos=transition_pos,
                           font_color="black",
                           ax=axes[1])
    plt.axis('equal')
    plt.show()


def find_trip(dump, load, road_network, paths_type='shortest', weight=None):

    ret = []
    if paths_type == 'shortest':
        shortest_path = nx.shortest_path(road_network,
                                         source=dump,
                                         target=load,
                                         weight=weight,
                                         method="dijkstra")
        ret.append(shortest_path)

    elif paths_type == 'all_simple':
        all_simple_paths = nx.all_simple_paths(road_network, dump, load)
        ret = list(all_simple_paths)  # because all_simple_paths return a generator
    else:
        print(f'Unknown paths type. {paths_type}.')
        raise ValueError

    return ret


if __name__ == '__main__':
    main()
