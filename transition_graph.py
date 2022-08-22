from enum import Enum, auto
from collections import deque
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


class Location:
    def __init__(self, name: str, x: float, y: float, loc_type: LocationType, activity_time):
        self.name: str = name
        self.pos: tuple[float, float] = (x, y)
        self.loc_type: LocationType = loc_type

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.__str__()


g_counter = 1


class Node:
    def __init__(self, loc_name: str, dst_nodes: list, transition_state: Enum):
        global g_counter
        self._node_name = f'v_{g_counter}'
        self._loc_name: str = loc_name
        self._dst_nodes: list[Node] = dst_nodes
        self._transition_state: Enum = transition_state
        g_counter += 1

    @property
    def loc_name(self):
        return self._loc_name

    @property
    def transition_state(self):
        return self._transition_state

    def __str__(self):
        return f'{self._node_name}, {self._loc_name}, {self._transition_state}, {self._dst_nodes}'

    def __repr__(self):
        return f'{self._node_name}'

    def is_same(self, other):
        return self._loc_name == other.loc_name and self._transition_state == other.transition_state

    def is_in(self, others):
        for other in others:
            if self.is_same(other):
                return True
        return False

    # Don't use them if you want to use networkx!
    # def __eq__(self, other):
    #     return self._loc_name == other.loc_name and self._transition_state == other.transition_state
    #
    # def __ne__(self, other):
    #     return self._loc_name != other.loc_name or self._transition_state != other.transition_state
    #
    # def __hash__(self):
    #     return id(self)

    def set_dst_nodes(self, dst_nodes: list):
        self._dst_nodes = dst_nodes


def init_transition_state(loading_loc_list):
    state_dict = {}
    for loc in loading_loc_list:
        state_dict['LOADED_' + loc] = auto()
        state_dict['LOADED_S_' + loc] = auto()
        state_dict['EMPTY_S_' + loc] = auto()
    state_dict['EMPTY'] = auto()
    transition_state = Enum('TransState', state_dict)
    return transition_state


def main():
    global g_counter

    # Create a networkx graph object
    road_network = nx.DiGraph()

    data = {
        'A': (0, 0, LocationType.CRUSHER, 90),
        'B': (1.73, 1, LocationType.ORE_LOAD, 120),
        'C': (1.73, -1, LocationType.ORE_LOAD, 120),
        'D': (2. / 1.73, 0, LocationType.INTSCT, 0),
    }
    loc_dict = {}
    for name, attr in data.items():
        loc = Location(name, attr[0], attr[1], attr[2], attr[3])
        loc_dict[name] = loc

    # Add edges to to the graph object
    # Each tuple represents an edge between two nodes

    road_network.add_edges_from([
        (loc_dict['A'], loc_dict['D']),
        (loc_dict['D'], loc_dict['A']),
        (loc_dict['B'], loc_dict['D']),
        (loc_dict['D'], loc_dict['B']),
        (loc_dict['C'], loc_dict['D']),
        (loc_dict['D'], loc_dict['C']),
    ])
    for edge in road_network.edges:
        road_network.edges[edge]["EMPTY_TRAVEL_TIME"] = 40
        road_network.edges[edge]["LOADED_TRAVEL_TIME"] = 60

    pos = {}
    for name in loc_dict.keys():
        pos[loc_dict[name]] = (loc_dict[name].pos[0], loc_dict[name].pos[1])

    # Draw the resulting road_network
    # fig, ax = plt.subplots()
    # dump_list = [loc for loc in road_network.nodes if
    #              loc.loc_type == LocationType.CRUSHER or loc.loc_type == LocationType.WST_DUMP]
    # load_list = [loc for loc in road_network.nodes if
    #              loc.loc_type == LocationType.ORE_LOAD or loc.loc_type == LocationType.WST_LOAD]
    # int_list = [loc for loc in road_network.nodes if loc.loc_type == LocationType.INTSCT]
    # nx.draw_networkx_nodes(road_network, pos=pos, nodelist=load_list, node_size=500, node_color='lightgreen',
    #                        edgecolors='gray')
    # nx.draw_networkx_nodes(road_network, pos=pos, nodelist=dump_list, node_size=500, node_color='red',
    #                        edgecolors='gray')
    # nx.draw_networkx_nodes(road_network, pos=pos, nodelist=int_list, node_size=500, node_color='white',
    #                        edgecolors='gray')
    # nx.draw_networkx_labels(road_network, pos=pos, font_color="black")
    # nx.draw_networkx_edges(road_network, pos=pos, width=1.0, alpha=1.0, edge_color="black", arrowsize=10.0,
    #                        connectionstyle='arc3, rad = 0.1', ax=ax)
    # plt.axis('equal')
    # plt.show()

    # Transition graph
    # NOTE: start from the sortest paths version for the simplicity.

    # (i) search cycles
    cycle_list = []
    dump_list = [loc for loc in road_network.nodes if
                 loc.loc_type == LocationType.CRUSHER or loc.loc_type == LocationType.WST_DUMP]
    load_list = [loc for loc in road_network.nodes if
                 loc.loc_type == LocationType.ORE_LOAD or loc.loc_type == LocationType.WST_LOAD]

    # DEBUG: fix a start loc
    dump = dump_list[0]

    loading_loc_name_list = [e.name for e in load_list]
    transition_state = init_transition_state(loading_loc_name_list)

    transition_graph = nx.DiGraph()
    transition_nodes = []
    transition_graph = nx.DiGraph()

    for load in load_list:

        cycle_path = []
        shortest_path = nx.shortest_path(road_network, source=dump, target=load, weight='EMPTY_TRAVEL_TIME',
                                         method="dijkstra")
        cycle_path.extend(shortest_path)
        shortest_path = nx.shortest_path(road_network, source=load, target=dump, weight='LOAD_TRAVEL_TIME',
                                         method="dijkstra")
        cycle_path.extend(shortest_path[1:-1])
        cycle_list.append(cycle_path)

        print(cycle_path)
        transition_cycle = []
        transition_cycle = []


        p_loc = None
        is_loaded = False
        loaded_loc = None
        for idx, loc in enumerate(cycle_path):


            if loc.loc_type == LocationType.WST_DUMP or loc.loc_type == LocationType.CRUSHER:
                transition_cycle.append(loc)

            if loc.loc_type == LocationType.WST_LOAD or loc.loc_type == LocationType.ORE_LOAD:
                is_loaded = True
                loaded_loc = loc
                transition_cycle.append(loc)

            if loc.loc_type == LocationType.INTSCT:
                if is_loaded:
                    new_loc = Location(loc.name + '_L_' + str(loaded_loc), loc.pos[0], loc.pos[1], LocationType.INTSCT, 0)
                else:
                    new_loc = Location(loc.name + '_E', loc.pos[0], loc.pos[1], LocationType.INTSCT, 0)

                transition_cycle.append(new_loc)

        for idx, loc in enumerate(transition_cycle):

            if p_loc is None:
                p_loc = transition_cycle[-1]
            else:
                p_loc = transition_cycle[idx-1]

            transition_graph.add_edge(p_loc, loc)


    del_locs = []
    for loc in transition_graph.nodes:
        print(loc, id(loc))
        if loc.name == 'D_E':
            del_locs.append(loc)

    del_arc = list(transition_graph.out_edges(del_locs[0]))[0]
    print(del_arc)
    transition_graph.remove_node(del_locs[0])
    transition_graph.add_edge(del_locs[1], del_arc[1])


    #Q transition_graph.remove_node(del_loc)
    nx.draw_networkx(transition_graph)
    plt.show()

    #     for idx, loc in enumerate(cycle_path):


    #         if loc.loc_type == LocationType.CRUSHER:
    #
    #             new_node = Node(loc.name, [], transition_state['EMPTY'])
    #             if not new_node.is_in(transition_nodes):
    #                 transition_nodes.append(new_node)
    #
    #                 # this is the first node in the transition_graph
    #                 transition_graph.add_node(new_node)
    #
    #             else:
    #                 g_counter -= 1
    #                 print(f'{new_node} is already in the transition nodes.')
    #
    #             new_node = Node(loc.name, [], transition_state[f'LOADED_{load}'])
    #             if not new_node.is_in(transition_nodes):
    #                 transition_nodes.append(new_node)
    #
    #                 # this is the second node in the transition graph
    #                 transition_graph.add_node(new_node)
    #                 # now we can add an arc
    #                 p_node = transition_nodes[0]
    #                 transition_graph.add_edge(p_node, new_node)
    #
    #             else:
    #                 g_counter -= 1
    #                 print(f'{new_node} is already in the transition nodes.')
    #
    #             new_node = Node(loc.name, [], transition_state[f'EMPTY_S_{load}'])
    #             if not new_node.is_in(transition_nodes):
    #                 transition_nodes.append(new_node)
    #
    #                 # this is the third one. Find the leaf and connect with a new node.
    #                 leaf = get_leaf(transition_graph)
    #                 transition_graph.add_edge(leaf, new_node)
    #             else:
    #                 g_counter -= 1
    #                 print(f'{new_node} is already in the transition nodes.')
    #
    #
    #         elif loc.loc_type == LocationType.INTSCT:
    #             if is_loaded:
    #                 new_node = Node(loc.name, [], transition_state[f'LOADED_{load}'])
    #                 if not new_node.is_in(transition_nodes):
    #                     transition_nodes.append(new_node)
    #                     leaf = get_leaf(transition_graph)
    #                     transition_graph.add_edge(leaf, new_node)
    #
    #                 else:
    #                     g_counter -= 1
    #                     print(f'{new_node} is already in the transition nodes.')
    #             else:
    #                 new_node = Node(loc.name, [], transition_state['EMPTY'])
    #                 if not new_node.is_in(transition_nodes):
    #                     transition_nodes.append(new_node)
    #                     leaf = get_leaf(transition_graph)
    #                     transition_graph.add_edge(leaf, new_node)
    #
    #                 else:
    #                     g_counter -= 1
    #                     print(f'{new_node} is already in the transition nodes.')
    #
    #
    #         elif loc.loc_type == LocationType.ORE_LOAD:
    #
    #             new_node = Node(loc.name, [], transition_state['EMPTY'])
    #             if not new_node.is_in(transition_nodes):
    #                 transition_nodes.append(new_node)
    #                 leaf = get_leaf(transition_graph)
    #                 transition_graph.add_edge(leaf, new_node)
    #
    #             else:
    #                 g_counter -= 1
    #                 print(f'{new_node} is already in the transition nodes.')
    #
    #             new_node = Node(loc.name, [], transition_state[f'LOADED_S_{load}'])
    #             if not new_node.is_in(transition_nodes):
    #                 transition_nodes.append(new_node)
    #                 leaf = get_leaf(transition_graph)
    #                 transition_graph.add_edge(leaf, new_node)
    #
    #             else:
    #                 g_counter -= 1
    #                 print(f'{new_node} is already in the transition nodes.')
    #
    #             new_node = Node(loc.name, [], transition_state[f'LOADED_{load}'])
    #             if not new_node.is_in(transition_nodes):
    #                 transition_nodes.append(new_node)
    #                 leaf = get_leaf(transition_graph)
    #                 transition_graph.add_edge(leaf, new_node)
    #
    #             else:
    #                 g_counter -= 1
    #                 print(f'{new_node} is already in the transition nodes.')
    #
    #             is_loaded = True
    #
    #     # build a transition_graph
    #     print(f'{transition_cycle=}')
    #     print(f'{transition_nodes=}')
    #
    #     # for idx, _ in enumerate(transition_cycle):
    #     #     node_from = transition_cycle[idx]
    #     #     node_to = transition_cycle[(idx+1) % len(transition_cycle)]
    #     #     transition_graph.add_edge(node_from, node_to)
    #
    # print(transition_graph)
    #
    # for node in transition_graph.nodes:
    #     print(node)
    #
    # nx.draw_networkx(transition_graph)
    # plt.show()


def get_leaf(transition_graph):
    leaves = [x for x in transition_graph.nodes() if
              transition_graph.out_degree(x) == 0 and transition_graph.in_degree(x) == 1]
    if len(leaves) > 1:
        print(transition_graph)
        print(f'{leaves=}')

        # find existing leaf and return another one
        leaf = leaves[-1]  # FIXME: Does it work?
        return leaf
    else:
        leaf = leaves[0]
        return leaf


if __name__ == '__main__':
    main()
