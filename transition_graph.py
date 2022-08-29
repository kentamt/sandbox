import sys
from enum import Enum, auto
import networkx as nx
from matplotlib import pyplot as plt
import matplotlib
from tabulate import tabulate

from logger import *

matplotlib.use('TkAgg')


class LocationType(Enum):
    ORE_LOAD = auto(),
    ORE_DUMP = auto(),
    WST_LOAD = auto(),
    WST_DUMP = auto(),
    INTSCT = auto()


class TransitionType(Enum):
    TRAVELING = auto(),
    ACTIVITY = auto(),
    REWARDING = auto()


class TransitionNode:
    def __init__(self, name: str, x: float, y: float, loc_type: LocationType, activity_time,
                 loc_name, state_name, star=False, is_loaded=None,
                 loaded_loc=None):
        self.name: str = name
        self.pos: tuple[float, float] = (x, y)
        self.loc_type: LocationType = loc_type
        self.activity_time = activity_time
        self.loc_name = loc_name
        self.state_name = state_name
        self.star = star
        self.is_loaded = is_loaded
        self.loaded_loc = loaded_loc

        self.is_activity_location = False

        # TODO: to be named as NodeLabels?
        self.u_ke = None

    def __str__(self):
        return f'{self.name}'

    def __repr__(self):
        return self.__str__()


class TransitionLabel:
    def __init__(self, l, f, k, a, r):
        """
        l = duration (distance) of transition e
        f = distance between a preceding vehicle of e
        k = road index
        a = reward bucket index
        r = reward bucket
        """
        self.l = None
        self.f = None
        self.k = None
        self.a = None
        self.r = None

    def __str__(self):
        return f'[l={self.l}, f={self.f}, k={self.k}, a={self.a}, r={self.r}]'


class RoadNetwork:
    """"""

    def __init__(self, input_type='1'):
        """Constructor for RoadNetwork"""
        self.R = None
        self.__build(input_type)

    def __build(self, input_type: str):
        data = dict()
        if input_type == '1':
            data = {
                'A': (0, 0, LocationType.ORE_DUMP, 90),
                'B': (1.73, 1, LocationType.ORE_LOAD, 120),
                'C': (1.73, -1, LocationType.ORE_LOAD, 120),
                'D': (2. / 1.73, 0, LocationType.INTSCT, 0),
            }
        elif input_type == '2':
            data = {
                'A': (0, 0, LocationType.ORE_DUMP, 90),
                'B': (1.73, 1, LocationType.ORE_LOAD, 120),
                'C': (1.73, -1, LocationType.ORE_LOAD, 120),
                'D': (2. / 1.73, 0, LocationType.INTSCT, 0),
                'E': (2., 0, LocationType.INTSCT, 0),
            }
        elif input_type == '3':
            data = {
                'A': (0.0, 0, LocationType.ORE_DUMP, 90),
                'B': (3.0, 1, LocationType.ORE_LOAD, 120),
                'C': (3.0, -1, LocationType.ORE_LOAD, 120),
                'D': (1.5, 0, LocationType.INTSCT, 0),
                'E': (2.5, 0, LocationType.INTSCT, 0),
                'F': (2.0, 1, LocationType.INTSCT, 0),
            }
        elif input_type == '4':
            data = {
                'A': (0, 1, LocationType.ORE_DUMP, 120),
                'E': (0, -1, LocationType.WST_DUMP, 120),
                'B': (2.0, 1, LocationType.ORE_LOAD, 120),
                'C': (2.0, -1, LocationType.WST_LOAD, 120),
                'D': (1.0, 0, LocationType.INTSCT, 0),
            }

        elif input_type == '5':
            data = {
                'A': (0.0, 0, LocationType.ORE_DUMP, 90),
                'B': (3.0, 1, LocationType.ORE_LOAD, 120),
                'C': (3.0, -1, LocationType.ORE_LOAD, 120),
                'D': (1.5, 0, LocationType.INTSCT, 0),
                'E': (2.5, 0, LocationType.INTSCT, 0),
                'F': (2.0, 1, LocationType.INTSCT, 0),
                'G': (4.0, -1, LocationType.WST_LOAD, 120),
                'H': (0.0, -1, LocationType.WST_DUMP, 90),
            }
        else:
            logger.error('Unknown input number.')
            raise ValueError

        loc_dict = {}
        for name, attr in data.items():
            loc = TransitionNode(name, attr[0], attr[1], attr[2], attr[3], name, None)
            loc_dict[name] = loc

        # Create a networkx graph object
        road_network = nx.DiGraph()
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
                (loc_dict['F'], loc_dict['E']),
                (loc_dict['E'], loc_dict['F']),
                (loc_dict['C'], loc_dict['D']),
                (loc_dict['D'], loc_dict['C']),
            ])
        elif input_type == '4':
            road_network.add_edges_from([
                (loc_dict['A'], loc_dict['D']),
                (loc_dict['D'], loc_dict['A']),
                (loc_dict['E'], loc_dict['D']),
                (loc_dict['D'], loc_dict['E']),
                (loc_dict['B'], loc_dict['D']),
                (loc_dict['D'], loc_dict['B']),
                (loc_dict['C'], loc_dict['D']),
                (loc_dict['D'], loc_dict['C']),
            ])
        elif input_type == '5':
            road_network.add_edges_from([
                (loc_dict['A'], loc_dict['D']),
                (loc_dict['D'], loc_dict['A']),
                (loc_dict['D'], loc_dict['E']),
                (loc_dict['E'], loc_dict['D']),
                (loc_dict['B'], loc_dict['E']),
                (loc_dict['E'], loc_dict['B']),
                (loc_dict['C'], loc_dict['D']),
                (loc_dict['D'], loc_dict['C']),

                (loc_dict['E'], loc_dict['G']),
                (loc_dict['G'], loc_dict['E']),
                (loc_dict['E'], loc_dict['H']),
                (loc_dict['H'], loc_dict['E']),
            ])

        for edge in road_network.edges:
            road_network.edges[edge]["EMPTY_TRAVEL_TIME"] = 40  # DEBUG: should be determined by distances
            road_network.edges[edge]["LOADED_TRAVEL_TIME"] = 60

        self.R = road_network


class TransitionGraph:
    def __init__(self, road_network: nx.DiGraph):
        self.R = road_network
        self.G = None
        self.distance_dict: dict = dict()

        # build network
        self.G = self.__build()
        # add transition labels
        self.__set_labels_edges()

    def show_labels_table(self):
        table = []
        for edge in self.G.edges:
            node_fr, node_to = edge
            l = self.G.edges[edge]['transition']
            row = [f'{node_fr.loc_name} ==> {node_to.loc_name}', l.l, l.f, l.k, l.a, l.r]
            table.append(row)
        print(tabulate(table, headers=["edge", "l_e", 'f_e', 'k_e', 'a_e', 'r_e'], tablefmt="github"))

    def show(self):
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        pos = {}
        for loc in self.R.nodes:
            pos[loc] = loc.pos

        for loc_type in [LocationType.ORE_LOAD, LocationType.WST_LOAD]:
            load_list = self.__get_load_list(loc_type)
            nx.draw_networkx_nodes(self.R,
                                   pos=pos,
                                   nodelist=load_list,
                                   node_size=500,
                                   node_color='lightgreen',
                                   edgecolors='gray',
                                   ax=axes[0])

        for loc_type in [LocationType.ORE_DUMP, LocationType.WST_DUMP]:
            dump_list = self.__get_dump_list(loc_type)
            nx.draw_networkx_nodes(self.R,
                                   pos=pos,
                                   nodelist=dump_list,
                                   node_size=500,
                                   node_color='red',
                                   edgecolors='gray',
                                   ax=axes[0])

        int_list = self.__get_intersection_list()
        nx.draw_networkx_nodes(self.R,
                               pos=pos,
                               nodelist=int_list,
                               node_size=500,
                               node_color='white',
                               edgecolors='gray',
                               ax=axes[0])
        nx.draw_networkx_labels(self.R,
                                pos=pos,
                                font_color="black",
                                ax=axes[0])
        nx.draw_networkx_edges(self.R,
                               pos=pos,
                               width=1.0,
                               alpha=1.0,
                               edge_color="black",
                               arrowsize=10.0,
                               connectionstyle='arc3, rad = 0.1',
                               ax=axes[0])
        axes[0].set_title('Road Network')



        transition_pos = nx.kamada_kawai_layout(self.G)
        label_pos = {}
        for loc, pos in transition_pos.items():
            label_pos[loc] = (pos[0], pos[1] + 0.1)

        for loc_type in [LocationType.ORE_LOAD, LocationType.WST_LOAD]:
            transition_load_list = [loc for loc in self.G.nodes if
                                    loc.loc_type == loc_type]  # or loc.loc_type == LocationType.WST_LOAD]
            nx.draw_networkx_nodes(self.G,
                                   pos=transition_pos,
                                   nodelist=transition_load_list,
                                   node_color='lightgreen',
                                   edgecolors='gray',
                                   node_size=200,
                                   ax=axes[1])
        for loc_type in [LocationType.ORE_DUMP, LocationType.WST_DUMP]:
            transition_dump_list = [loc for loc in self.G.nodes if
                                    loc.loc_type == loc_type]  # or loc.loc_type == LocationType.WST_DUMP]
            nx.draw_networkx_nodes(self.G,
                                   pos=transition_pos,
                                   nodelist=transition_dump_list,
                                   node_color='red',
                                   edgecolors='gray',
                                   node_size=200,
                                   ax=axes[1])

        transition_intsct_list = [loc for loc in self.G.nodes if
                                  loc.loc_type == LocationType.INTSCT]

        nx.draw_networkx_nodes(self.G,
                               pos=transition_pos,
                               nodelist=transition_intsct_list,
                               node_color='white',
                               edgecolors='gray',
                               node_size=200,
                               ax=axes[1])

        nx.draw_networkx_edges(self.G,
                               pos=transition_pos,
                               arrowstyle='->',
                               arrowsize=20,
                               edge_color='black',
                               ax=axes[1])

        nx.draw_networkx_labels(self.G,
                                pos=label_pos,
                                font_color="black",
                                font_size=8,
                                ax=axes[1])
        axes[1].set_title('Transition Graph')
        plt.axis('equal')
        plt.show()

    def get_loc_names(self, loc_type=None):
        """
        TODO: now there are only two discrete states: EMPTY and LOAD,
        TODO: which are corresponding to CRUSHER and LOADING locations.
        TODO: in the future, three discrete states: EMPTY, ORE and WASTE should be used.
        """

        ret: list[str] = []
        node: TransitionNode
        if loc_type is None:
            for node in self.R.nodes:
                ret.append(node.loc_name)

        elif loc_type == 'loading':
            for node in self.R.nodes:
                if node.loc_type == LocationType.ORE_LOAD:
                    ret.append(node.loc_name)

        elif loc_type == 'dumping':
            for node in self.R.nodes:
                if node.loc_type == LocationType.ORE_DUMP:
                    ret.append(node.loc_name)

        elif loc_type == 'intersection':
            for node in self.R.nodes:
                if node.loc_type == LocationType.INTSCT:
                    ret.append(node.loc_name)

        return ret

    def find_node(self, loc_name: str, is_loaded: bool, star: bool):

        node: TransitionNode
        for node in self.G.nodes:
            if node.loc_name == loc_name and node.is_loaded == is_loaded and node.star == star:
                return node

        return None

    # -----------------------------------------------------------------------
    # private methods
    # -----------------------------------------------------------------------
    def __find_trip(self, dump, load, paths_type='shortest', weight=None):

        ret = []
        if paths_type == 'shortest':
            shortest_path = nx.shortest_path(self.R,
                                             source=dump,
                                             target=load,
                                             weight=weight,
                                             method="dijkstra")
            ret.append(shortest_path)

        elif paths_type == 'all_simple':
            all_simple_paths = nx.all_simple_paths(self.R, dump, load)
            ret = list(all_simple_paths)  # because all_simple_paths return a generator
        else:
            logger.error(f'Unknown paths type. {paths_type}.')
            raise ValueError

        return ret

    def __get_cycle_path_list(self, cycle_path_list, dump, load):
        outward_paths = self.__find_trip(dump, load, paths_type='all_simple')
        return_paths = self.__find_trip(load, dump, paths_type='all_simple')
        for outward_path in outward_paths:
            for return_path in return_paths:
                cycle_path: list[TransitionNode] = []
                # concat the outward and return trip. remove the end of the path because we will add the return trip
                cycle_path.extend(outward_path[:-1])
                cycle_path.extend(return_path)
                cycle_path_list.append(cycle_path)
        return cycle_path_list

    def __find_duplicated_locs(self):
        duplicated_locs = []
        visited_id = []
        for loc1 in self.G.nodes:
            visited_id.append(id(loc1))
            for loc2 in self.G.nodes:
                if id(loc2) in visited_id:
                    continue

                if id(loc1) != id(loc2) and loc1.name == loc2.name:
                    duplicated_locs.append(loc1)
        return duplicated_locs

    def __get_intersection_list(self):
        int_list = [loc for loc in self.R.nodes if loc.loc_type == LocationType.INTSCT]
        return int_list

    def __get_load_list(self, loc_type):
        load_list = [loc for loc in self.R.nodes if
                     loc.loc_type == loc_type]  # or loc.loc_type == LocationType.WST_LOAD]
        return load_list

    def __get_dump_list(self, loc_type):
        dump_list = [loc for loc in self.R.nodes if
                     loc.loc_type == loc_type]  # or loc.loc_type == LocationType.WST_DUMP]
        return dump_list

    def __init_distance_dict(self):
        """
        init physical distances (duration) according to the road network
        :return:
        """

        for edge in self.R.edges:
            loc_fr: TransitionNode = edge[0]
            loc_to: TransitionNode = edge[1]
            if not loc_fr.loc_name in self.distance_dict.keys():
                self.distance_dict[loc_fr.loc_name] = {}
            else:
                if loc_fr.is_loaded:
                    travel_time = edge['LOADED_TRAVEL_TIME']
                else:
                    travel_time = edge['EMPTY_TRAVEL_TIME']
                self.distance_dict[loc_fr.loc_name][loc_to.loc_name] = travel_time

    def __build(self):

        self.G = nx.DiGraph()

        load_unload_type_pair = [(LocationType.ORE_LOAD, LocationType.ORE_DUMP),
                                 (LocationType.WST_LOAD, LocationType.WST_DUMP)]

        for type_pair in load_unload_type_pair:

            # dump_type = LocationType.ORE_DUMP
            # load_type = LocationType.ORE_LOAD
            load_type = type_pair[0]
            dump_type = type_pair[1]

            dump_list = self.__get_dump_list(dump_type)
            load_list = self.__get_load_list(load_type)

            print(dump_type)
            print(f'{dump_list=}')
            print(load_type)
            print(f'{load_list=}')
            # exit()

            # search for cycles from dumping point to loading point
            for dump in dump_list:
                cycle_path_list = []
                for load in load_list:
                    cycle_path_list = self.__get_cycle_path_list(cycle_path_list, dump, load)

                for cycle_path in cycle_path_list:

                    # a cycle in a transition graph
                    transition_cycle = []

                    p_loc = None
                    is_loaded = False
                    loaded_loc = None
                    for idx, loc in enumerate(cycle_path):

                        # For loadings. They have three types of nodes:
                        # (1) EMPTY, (2) LOADED*_{loc.name} and (3) LOADED_{loc.name}
                        if loc.loc_type == dump_type: # LocationType.ORE_DUMP:  # or loc.loc_type == LocationType.WST_DUMP:

                            if is_loaded:
                                # name, x, y, loc_type, activity_time, loaded, star, loc_name):
                                state = "LOADED_" + loaded_loc.loc_name
                                new_loc = TransitionNode(loc.name + '_LOAD_' + str(loaded_loc), loc.pos[0], loc.pos[1],
                                                         loc.loc_type,
                                                         loc.activity_time, loc.name, state,
                                                         is_loaded=True, loaded_loc=loaded_loc)
                                transition_cycle.append(new_loc)

                                state = "EMPTY_S_" + loaded_loc.loc_name
                                new_loc = TransitionNode(loc.name + '_EMPTY_S_' + str(loaded_loc), loc.pos[0], loc.pos[1],
                                                         loc.loc_type,
                                                         loc.activity_time, loc.name, state, star=True,
                                                         is_loaded=False, loaded_loc=loaded_loc)
                                transition_cycle.append(new_loc)
                            else:
                                state = "EMPTY"
                                new_loc = TransitionNode(loc.name + '_EMPTY', loc.pos[0], loc.pos[1], loc.loc_type,
                                                         loc.activity_time, loc.name, state,
                                                         is_loaded=False, loaded_loc=loaded_loc)
                                transition_cycle.append(new_loc)

                        # For dumpings. They have three types of nodes:
                        # (1) LOADED_{loc.name}, (2) EMPTY*_{loc.name} and (3) EMPTY
                        if loc.loc_type ==  load_type: #  LocationType.ORE_LOAD:  # or loc.loc_type == LocationType.WST_LOAD:
                            is_loaded = True
                            loaded_loc = loc
                            state = "EMPTY"
                            new_loc = TransitionNode(loc.name + '_EMPTY', loc.pos[0], loc.pos[1], loc.loc_type,
                                                     loc.activity_time, loc.name, state,
                                                     is_loaded=False, loaded_loc=loaded_loc)
                            transition_cycle.append(new_loc)

                            state = 'LOADED_S_' + loaded_loc.loc_name
                            new_loc = TransitionNode(loc.name + '_LOAD_S_' + loaded_loc.name, loc.pos[0], loc.pos[1],
                                                     loc.loc_type,
                                                     loc.activity_time, loc.name, state, star=True,
                                                     is_loaded=True, loaded_loc=loaded_loc)
                            transition_cycle.append(new_loc)

                            state = 'LOADED_' + loaded_loc.loc_name
                            new_loc = TransitionNode(loc.name + '_LOAD_' + loaded_loc.loc_name, loc.pos[0], loc.pos[1],
                                                     loc.loc_type,
                                                     loc.activity_time, loc.name, state,
                                                     is_loaded=True, loaded_loc=loaded_loc)
                            transition_cycle.append(new_loc)

                        # For intersections. They have two types.
                        # (1) LOADED_{loc.name} and (2) EMPTY
                        if loc.loc_type == LocationType.INTSCT:
                            if is_loaded:
                                state = 'LOADED_' + loaded_loc.loc_name
                                new_loc = TransitionNode(loc.name + '_LOAD_' + loaded_loc.loc_name, loc.pos[0], loc.pos[1],
                                                         LocationType.INTSCT,
                                                         loc.activity_time, loc.name, state,
                                                         is_loaded=True, loaded_loc=loaded_loc)
                            else:
                                state = 'EMPTY'
                                new_loc = TransitionNode(loc.name + '_EMPTY', loc.pos[0], loc.pos[1], LocationType.INTSCT,
                                                         loc.activity_time, loc.name, state,
                                                         is_loaded=False, loaded_loc=loaded_loc)
                            transition_cycle.append(new_loc)

                    logger.debug(f'{transition_cycle=}')

                    # build a transition graph
                    for idx, loc in enumerate(transition_cycle):
                        if p_loc is None:
                            p_loc = transition_cycle[-1]
                        else:
                            p_loc = transition_cycle[idx - 1]
                        self.G.add_edge(p_loc, loc)
            # Find nodes which have the same names
            duplicated_locs = self.__find_duplicated_locs()
            # Delete and reconnect nodes
            for loc in duplicated_locs:

                # find duplicated nodes witch have the same name in a graph
                loc_same_names = []
                for _loc in self.G.nodes:
                    if id(_loc) != id(loc) and _loc.name == loc.name:
                        loc_same_names.append(_loc)

                # delete the target node and reconnect to the duplicated node.
                for loc_same_name in loc_same_names:

                    # find in-edges and reconnect all
                    in_arcs = list(self.G.in_edges(loc_same_name))
                    for in_arc in in_arcs:
                        loc_from = in_arc[0]
                        loc_to = loc
                        self.G.add_edge(loc_from, loc_to)

                    # find in-edges and reconnect all
                    out_arcs = list(self.G.out_edges(loc_same_name))
                    for out_arc in out_arcs:
                        loc_to = out_arc[1]
                        loc_from = loc
                        self.G.add_edge(loc_from, loc_to)

                    logger.debug(f'remove {loc_same_name}')
                    self.G.remove_node(loc_same_name)
        return self.G

    def __check_transition_type(self, node_fr, node_to):
        if node_fr.loc_name != node_to.loc_name:
            return TransitionType.TRAVELING
        elif node_fr.loc_name == node_to.loc_name and node_to.star:
            return TransitionType.ACTIVITY
        elif node_fr.star and not node_to.star:
            return TransitionType.REWARDING
        else:
            logger.debug('Unknown combination of nodes')
            raise ValueError

    def __set_labels_edges(self):
        """
        l = duration (distance) of transition e
        f = distance between a preceding vehicle of e
        k = road index
        a = reward bucket index
        r = reward bucket
        """

        for edge in self.G.edges:
            label = TransitionLabel(0, 0, 0, 0, 0)
            node_from: TransitionNode = edge[0]
            node_to: TransitionNode = edge[1]
            transition_type = self.__check_transition_type(node_from, node_to)

            if transition_type == TransitionType.TRAVELING:
                if node_from.is_loaded:
                    label.l = 60
                    label.f = 15
                else:
                    label.l = 40
                    label.f = 10
                label.k = (node_from.loc_name, node_to.loc_name)
                label.r = 0
                label.a = None

            elif transition_type == TransitionType.ACTIVITY:
                if node_to.is_loaded:
                    label.l = 120
                    label.f = 120
                else:
                    label.l = 90
                    label.f = 90
                label.k = (node_from.loc_name, node_to.loc_name)  # same
                label.r = 0
                label.a = None

            elif transition_type == TransitionType.REWARDING:
                label.l = 0
                label.f = 0
                label.k = None
                label.r = 1
                if node_to.is_loaded:
                    label.a = (node_from.loc_name, None)
                else:
                    label.a = (node_from.loaded_loc.loc_name, node_to.loc_name)

            self.G.edges[edge]["transition"] = label


def main():
    input_type = sys.argv[1]

    # Road Network
    road_network = RoadNetwork(input_type)

    # Transition graph
    transition_graph = TransitionGraph(road_network.R)
    transition_graph.show()
    transition_graph.show_labels_table()

    # Pick up node
    print(transition_graph.find_node("A", False, False))
    print(transition_graph.find_node("A", False, True))
    print(transition_graph.find_node("A", True, True))
    print(transition_graph.find_node("C", True, True))

    # Pick up node at random
    # Find loading node

    print(transition_graph.get_loc_names())
    print(transition_graph.get_loc_names(loc_type='loading'))
    print(transition_graph.get_loc_names(loc_type='dumping'))
    print(transition_graph.get_loc_names(loc_type='intersection'))


if __name__ == '__main__':
    main()
