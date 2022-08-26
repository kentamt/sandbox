import numpy as np
from transition_graph import RoadNetwork, TransitionGraph

class ObjectiveFunction():
    """"""

    def __init__(self, discount_factor):
        """Constructor for ObjectiveFunction"""
        self.xi = discount_factor
        self.g = GoalFunction()

    def update(self, t):
        return 0 * self.xi * t

    def get_val(self):
        return 0

    @staticmethod
    def e(x):
        if x <= 0:
            return 1.0/4.0 - (x - 1.0/2.0)**2
        else:
            return np.sqrt(x + 1.0 / 4.0) - 1.0/2.0

class GoalFunction():
    """
    return
    """

    def __init__(self, ):
        """Constructor for GoalFunction"""
        pass

    def get_val(self, a1, a2, t):

        if a2 is None:
            return 5 * t  # DEBUG: set value from the upper solution
        else:
            return 1 * t


class SystemState:
    """"""

    def __init__(self, transition_graph, num_trucks, num_locations, num_pairs):
        """Constructor for """
        self.G = transition_graph
        self.d = [0] * num_trucks  # a vertex id that the i-th truck is heading to.
        self.td = [0] * num_trucks  # an estimated time of arrival at the next node
        self.u = [0] * num_locations # a time that the i-th truck will finish/finished its task
        self.r = [0] * num_pairs  # total reward the i-th node has got
        self.o1 = 0 # objective value
        self.o2 = 0 # objective value
        self.t = 0  # the last point in time when the objective function was updated

        self.o = ObjectiveFunction(0.5)

    def __str__(self):
        ret = ""

        ret += "d=["
        for e in self.d:
            ret += f"{e} "
        ret = ret[:-1]
        ret += '], '

        ret += "td=["
        for e in self.td:
            ret += f"{e} "
        ret = ret[:-1]
        ret += '], '

        ret += "u=["
        for e in self.u:
            ret += f"{e} "
        ret = ret[:-1]
        ret += '], '

        ret += "r=["
        for e in self.r:
            ret += f"{e} "
        ret = ret[:-1]
        ret += '], '

        ret += f"o1={self.o1}, "
        ret += f"o2={self.o2}, "
        ret += f"t={self.t}, "


        return ret

    def init(self, t: float):
        """
        give trucks initial positions

        state doesn't include the current position of each truck.
        It only contains the destination.
        We give their origins as destination with estimated arrival time = 0

        """
        # All vehicle start from 'A' as empty

        # self.d =



    def transition(self, t: float):
        """"""
        pass
        i = np.argmin(self.td)
        # self.d[i] = label_e # the label of e ∈ E.
        # self.t[i] = np.max(self.td[i], u_ke + f_e)
        # self.u[i] = self.t[i]
        # self.r[i] = self.r[i] + r_e
        # self.o1 = self.o(t)
        # self.o2 = self.o2 + self.o.xi ** 1.0/ dt
        self.t = t

    def evaluate(self):
        """"""
        pass

    def show(self):
        """"""
        print(self)

def main():
    road_network = RoadNetwork('1')
    transition_graph = TransitionGraph(road_network.R)

    n = 3
    m = 5
    p = 2  # the number of load-unload pair
    s = SystemState(transition_graph, n, m, p)

    t = 0  # [sec]
    dt = 5.0  # [sec]

    s.init(t)
    for _ in range(10):
        s.transition(t)
        s.show()
        t += dt

    s.evaluate()

if __name__ == '__main__':
    main()
    print('EOP')