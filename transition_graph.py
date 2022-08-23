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


g_counter = 1


class Node:
    def __init__(self, loc_name: str, dst_nodes: list, transition_state: Enum,
                 loaded=False, load_loc=None, star=False):
        global g_counter
        self._node_name = f'v_{g_counter}'
        self._loc_name: str = loc_name
        self._dst_nodes: list[Node] = dst_nodes
        self._transition_state: Enum = transition_state
        self_loaded = loaded
        self.star = star  # star means a destination node
        g_counter += 1

    @property
    def loc_name(self):
        return self._loc_name

    @property
    def transition_state(self):
        return self._transition_state

    def __str__(self):
        # return f'{self._node_name}, {self._loc_name}, {self._transition_state}, {self._dst_nodes}'
        return f'{self._node_name}'

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

    # data = {
    #     'A': (0, 0, LocationType.CRUSHER, 90),
    #     'B': (1.73, 1, LocationType.ORE_LOAD, 120),
    #     'C': (1.73, -1, LocationType.ORE_LOAD, 120),
    #     'D': (2. / 1.73, 0, LocationType.INTSCT, 0),
    # }

    data = {
        'A': (0, 0, LocationType.CRUSHER, 90),
        'B': (1.73, 1, LocationType.ORE_LOAD, 120),
        'C': (1.73, -1, LocationType.ORE_LOAD, 120),
        'D': (2. / 1.73, 0, LocationType.INTSCT, 0),
        'E': (2., 0, LocationType.INTSCT, 0),
    }


    loc_dict = {}
    for name, attr in data.items():
        # name, x, y, loc_type, activity_time, loaded, star, loc_name):
        loc = Location(name, attr[0], attr[1], attr[2], attr[3], name, None)
        loc_dict[name] = loc

    # Add edges to to the graph object
    # Each tuple represents an edge between two nodes

    # road_network.add_edges_from([
    #     (loc_dict['A'], loc_dict['D']),
    #     (loc_dict['D'], loc_dict['A']),
    #     (loc_dict['B'], loc_dict['D']),
    #     (loc_dict['D'], loc_dict['B']),
    #     (loc_dict['C'], loc_dict['D']),
    #     (loc_dict['D'], loc_dict['C']),
    # ])

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

    for edge in road_network.edges:
        road_network.edges[edge]["EMPTY_TRAVEL_TIME"] = 40
        road_network.edges[edge]["LOADED_TRAVEL_TIME"] = 60

    pos = {}
    for name in loc_dict.keys():
        pos[loc_dict[name]] = (loc_dict[name].pos[0], loc_dict[name].pos[1])

    # Draw the resulting road_network
    fig, ax = plt.subplots()
    dump_list = [loc for loc in road_network.nodes if
                 loc.loc_type == LocationType.CRUSHER or loc.loc_type == LocationType.WST_DUMP]
    load_list = [loc for loc in road_network.nodes if
                 loc.loc_type == LocationType.ORE_LOAD or loc.loc_type == LocationType.WST_LOAD]
    int_list = [loc for loc in road_network.nodes if loc.loc_type == LocationType.INTSCT]
    nx.draw_networkx_nodes(road_network, pos=pos, nodelist=load_list, node_size=500, node_color='lightgreen',
                           edgecolors='gray')
    nx.draw_networkx_nodes(road_network, pos=pos, nodelist=dump_list, node_size=500, node_color='red',
                           edgecolors='gray')
    nx.draw_networkx_nodes(road_network, pos=pos, nodelist=int_list, node_size=500, node_color='white',
                           edgecolors='gray')
    nx.draw_networkx_labels(road_network, pos=pos, font_color="black")
    nx.draw_networkx_edges(road_network, pos=pos, width=1.0, alpha=1.0, edge_color="black", arrowsize=10.0,
                           connectionstyle='arc3, rad = 0.1', ax=ax)
    plt.axis('equal')
    plt.show()

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

    transition_nodes = []
    transition_graph = nx.DiGraph()

    for load in load_list:

        # a cycle in a road network
        cycle_path:list[Location] = []
        shortest_path = nx.shortest_path(road_network,
                                         source=dump,
                                         target=load,
                                         weight='EMPTY_TRAVEL_TIME',
                                         method="dijkstra")
        # add the outward trip. remove the end of the path because we will add the return trip
        cycle_path.extend(shortest_path[:-1])
        shortest_path = nx.shortest_path(road_network,
                                         source=load,
                                         target=dump,
                                         weight='LOAD_TRAVEL_TIME',
                                         method="dijkstra")
        # add the return trip
        cycle_path.extend(shortest_path)
        cycle_list.append(cycle_path)
        print(f'{cycle_path=}')

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
                    new_loc = Location(loc.name + '_L_' + str(loaded_loc), loc.pos[0], loc.pos[1], loc.loc_type,
                                       loc.activity_time, loc.name, state)
                    transition_cycle.append(new_loc)

                    state = "EMPTY_S_" + loaded_loc.loc_name
                    new_loc = Location(loc.name + '_E_S_' + str(loaded_loc), loc.pos[0], loc.pos[1], LocationType.INTSCT,
                                       loc.activity_time, loc.name, state, star=True)
                    transition_cycle.append(new_loc)
                else:
                    state = "EMPTY"
                    new_loc = Location(loc.name + '_E', loc.pos[0], loc.pos[1], loc.loc_type,
                                       loc.activity_time, loc.name, state)
                    transition_cycle.append(new_loc)

            # For dumpings. They have three types of nodes: (1) LOADED_{loc.name}, (2) EMPTY*_{loc.name} and (3) EMPTY
            if loc.loc_type == LocationType.WST_LOAD or loc.loc_type == LocationType.ORE_LOAD:
                is_loaded = True
                loaded_loc = loc
                state = "EMPTY"
                new_loc = Location(loc.name + '_E', loc.pos[0], loc.pos[1], loc.loc_type,
                                   loc.activity_time, loc.name, state)
                transition_cycle.append(new_loc)

                state = 'LOADED_S_' + loaded_loc.loc_name
                new_loc = Location(loc.name + '_L_S_' + loaded_loc.name, loc.pos[0], loc.pos[1], loc.loc_type,
                                   loc.activity_time, loc.name,  state, star=True)
                transition_cycle.append(new_loc)

                state = 'LOADED_' + loaded_loc.loc_name
                new_loc = Location(loc.name + '_L_' + loaded_loc.loc_name, loc.pos[0], loc.pos[1], loc.loc_type,
                                   loc.activity_time, loc.name, state)
                transition_cycle.append(new_loc)

            # For intersections. They have two types. (1) LOADED_{loc.name} and (2) EMPTY
            if loc.loc_type == LocationType.INTSCT:
                if is_loaded:
                    state = 'LOADED_' + loaded_loc.loc_name
                    new_loc = Location(loc.name + '_L_' + loaded_loc.loc_name, loc.pos[0], loc.pos[1], LocationType.INTSCT,
                                       loc.activity_time, loc.name, state)
                else:
                    state = 'EMPTY'
                    new_loc = Location(loc.name + '_E', loc.pos[0], loc.pos[1], LocationType.INTSCT,
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

    transition_pos = nx.spring_layout(transition_graph, seed=200)
    nx.draw_networkx_nodes(transition_graph, pos=transition_pos, node_color='royalblue', node_size=500)
    nx.draw_networkx_edges(transition_graph, pos=transition_pos,
                           arrowstyle='->',
                           arrowsize=20,
                           edge_color='salmon')
    nx.draw_networkx_labels(transition_graph, pos=transition_pos, font_color="black")
    plt.show()

    transition_graph2 = nx.DiGraph()
    transition_dict = {}
    for loc in transition_graph.nodes:
        loc: Location
        state = loc.state_name
        if loc.star:
            transition_dict[loc] = Node(loc.loc_name, [], transition_state[state], star=True)
        else:
            transition_dict[loc] = Node(loc.loc_name, [], transition_state[state], star=False)

    for arc in transition_graph.edges:
        node_from = transition_dict[arc[0]]
        node_to   = transition_dict[arc[1]]
        transition_graph2.add_edge(node_from, node_to)

    print(transition_graph2)
    transition_pos = nx.spring_layout(transition_graph2, seed=201)
    nx.draw_networkx_nodes(transition_graph2, pos=transition_pos, node_color='royalblue', node_size=500)
    nx.draw_networkx_edges(transition_graph2, pos=transition_pos,
                           arrowstyle='->',
                           arrowsize=20,
                           edge_color='salmon')
    nx.draw_networkx_labels(transition_graph2, pos=transition_pos, font_color="black")
    plt.show()



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
