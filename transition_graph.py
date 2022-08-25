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
    def __init__(self, name: str, x: float, y: float, loc_type: LocationType, activity_time,
                 loc_name, state_name, star=False, is_loaded=None, loaded_loc=None):
        self.name: str = name
        self.pos: tuple[float, float] = (x, y)
        self.loc_type: LocationType = loc_type
        self.activity_time = activity_time
        self.loc_name = loc_name
        self.state_name = state_name
        self.star = star
        self.is_loaded = is_loaded
        self.loaded_loc = loaded_loc

    def __str__(self):
        return self.name

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
        for edge in road_network.edges:
            road_network.edges[edge]["EMPTY_TRAVEL_TIME"] = 40  # DEBUG: should be determined by distances
            road_network.edges[edge]["LOADED_TRAVEL_TIME"] = 60

        self.R = road_network


class TransitionGraph:
    def __init__(self, road_network: nx.DiGraph):
        self.R = road_network
        self.G = None
        self.distance_dict: dict = dict()
        self.G = self.__build()
        self.__set_labels_edges()

    def find_trip(self, dump, load, paths_type='shortest', weight=None):

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
            print(f'Unknown paths type. {paths_type}.')
            raise ValueError

        return ret

    def get_cycle_path_list(self, cycle_path_list, dump, load):
        outward_paths = self.find_trip(dump, load, paths_type='all_simple')
        return_paths = self.find_trip(load, dump, paths_type='all_simple')
        for outward_path in outward_paths:
            for return_path in return_paths:
                cycle_path: list[TransitionNode] = []
                # concat the outward and return trip. remove the end of the path because we will add the return trip
                cycle_path.extend(outward_path[:-1])
                cycle_path.extend(return_path)
                cycle_path_list.append(cycle_path)
        return cycle_path_list

    def find_dupulicated_locs(self):
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

    def get_intersection_list(self):
        int_list = [loc for loc in self.R.nodes if loc.loc_type == LocationType.INTSCT]
        return int_list

    def get_load_list(self):
        load_list = [loc for loc in self.R.nodes if
                     loc.loc_type == LocationType.ORE_LOAD or loc.loc_type == LocationType.WST_LOAD]
        return load_list

    def get_dump_list(self):
        dump_list = [loc for loc in self.R.nodes if
                     loc.loc_type == LocationType.CRUSHER or loc.loc_type == LocationType.WST_DUMP]
        return dump_list

    def init_distance_dict(self):
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
        dump_list = self.get_dump_list()
        load_list = self.get_load_list()

        # search for cycles from dumping point to loading point
        for dump in dump_list:
            cycle_path_list = []
            for load in load_list:
                cycle_path_list = self.get_cycle_path_list(cycle_path_list, dump, load)

            for cycle_path in cycle_path_list:

                # a cycle in a transition graph
                transition_cycle = []

                p_loc = None
                is_loaded = False
                loaded_loc = None
                for idx, loc in enumerate(cycle_path):

                    # For loadings. They have three types of nodes:
                    # (1) EMPTY, (2) LOADED*_{loc.name} and (3) LOADED_{loc.name}
                    if loc.loc_type == LocationType.WST_DUMP or loc.loc_type == LocationType.CRUSHER:

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
                    if loc.loc_type == LocationType.WST_LOAD or loc.loc_type == LocationType.ORE_LOAD:
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

                print(f'{transition_cycle=}')

                # build a transition graph
                for idx, loc in enumerate(transition_cycle):
                    if p_loc is None:
                        p_loc = transition_cycle[-1]
                    else:
                        p_loc = transition_cycle[idx - 1]
                    self.G.add_edge(p_loc, loc)
        # Find nodes which have the same names
        duplicated_locs = self.find_dupulicated_locs()
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

                print(f'remove {loc_same_name}')
                self.G.remove_node(loc_same_name)
        return self.G

    def __set_labels_edges(self):
        label = TransitionLabel(0,0,0,0,0)
        for edge in self.G.edges:
            loc_from = edge[0]
            loc_to = edge[1]

            self.G.edges[edge]["transition"] = label

    def show(self):
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        pos = {}
        for loc in self.R.nodes:
            pos[loc] = loc.pos
        dump_list = self.get_dump_list()
        load_list = self.get_load_list()
        int_list = self.get_intersection_list()
        nx.draw_networkx_nodes(self.R,
                               pos=pos,
                               nodelist=load_list,
                               node_size=500,
                               node_color='lightgreen',
                               edgecolors='gray',
                               ax=axes[0])
        nx.draw_networkx_nodes(self.R,
                               pos=pos,
                               nodelist=dump_list,
                               node_size=500,
                               node_color='red',
                               edgecolors='gray',
                               ax=axes[0])
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
        transition_load_list = [loc for loc in self.G.nodes if
                                loc.loc_type == LocationType.ORE_LOAD or loc.loc_type == LocationType.WST_LOAD]
        transition_dump_list = [loc for loc in self.G.nodes if
                                loc.loc_type == LocationType.CRUSHER or loc.loc_type == LocationType.WST_DUMP]
        transition_intsct_list = [loc for loc in self.G.nodes if
                                  loc.loc_type == LocationType.INTSCT]
        # transition_pos = nx.spring_layout(self.G)
        transition_pos = nx.kamada_kawai_layout(self.G)
        label_pos = {}
        for loc, pos in transition_pos.items():
            label_pos[loc] = (pos[0], pos[1] + 0.1)
        nx.draw_networkx_nodes(self.G,
                               pos=transition_pos,
                               nodelist=transition_load_list,
                               node_color='lightgreen',
                               edgecolors='gray',
                               node_size=200,
                               ax=axes[1])
        nx.draw_networkx_nodes(self.G,
                               pos=transition_pos,
                               nodelist=transition_dump_list,
                               node_color='red',
                               edgecolors='gray',
                               node_size=200,
                               ax=axes[1])
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


def main():
    input_type = sys.argv[1]

    # Road Network
    road_network = RoadNetwork(input_type)

    # Transition graph
    transition_graph = TransitionGraph(road_network.R)
    transition_graph.show()


if __name__ == '__main__':
    main()
