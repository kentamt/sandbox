import networkx as nx


class Dog:
    def __init__(self, name):
        self.name  = name

    def say(self):
        print('Wan')

    def __eq__(self, other):
        return self.name == other.name

    def __ne__(self, other):
        return self.name != other.name

    def __hash__(self):
        return id(self)

if __name__ == '__main__':
    pochi = Dog('pochi')
    bell = Dog('bell')

    g = nx.DiGraph()

g.add_edge(pochi, bell)
print(g)