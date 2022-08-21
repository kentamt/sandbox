class node:
    def __init__(self, state):
        self.state = state
        self.w = 0  # value
        self.n = 0  # num trial
        self.child_nodes = None  #

    def evaluate(self):
        if self.state.end:
            value = -1 if self.state.lose else 0

            self.w += value
            self.n += 1

            return value

        if self.child_nodes is None:
            value = playout(self.state)

            self.w += value
            self.n += 1

            if self.n == 10:
                self.expand

            return value
        else:
            value = -self.next_child_node().evaluate()

            self.w += value
            self.n += 1

            return value

    def expand(self):
        self.child_nodes = tuple(node(self.state.next(action)) for action in self.state.legal_actions)

    def next_child_node(self):
        def ucb1_values():
            t = sum(map(attrgetter('n'), self.child_nodes))

            return tuple(-child_node.w / child_node.n + 2 * (2 * log(t) / child_node.n) ** 0.5 for child_node in
                         self.child_nodes)

        for child_node in self.child_nodes:
            if child_node.n == 0:
                return child_node

        ucb1_values = ucb1_values()

        return self.child_nodes[argmax(ucb1_values)]
