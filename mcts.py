from __future__ import division

import time
import math
import random
random.seed(1)

def randomPolicy(state):
    time_horizon = 10.0 * 60.0  # [sec]  FIXME: don't hard-code it here.
    sim_time_start = state.sim_time
    while not state.isTerminal():
        actions = state.getPossibleActions()
        action = random.choice(actions)
        #
        # try:
        #     actions = state.getPossibleActions()
        #     if len(actions)==1:
        #         action = actions[0]
        #     else:
        #         action = random.choice(actions)
        # except IndexError:
        #     raise Exception("Non-terminal state has no possible actions: " + str(state))

        state = state.takeAction(action)

        elapsed_sim_time = state.sim_time - sim_time_start
        if elapsed_sim_time > time_horizon:
            break
        # else:
        #     print(f'{elapsed_sim_time/60.0=}')

    return state.getReward()


def heuristicPolicy(state):
    raise NotImplementedError


class treeNode():
    def __init__(self, state, parent):
        self.state = state
        self.isTerminal = state.isTerminal()
        self.isFullyExpanded = self.isTerminal
        self.parent = parent
        self.numVisits = 0
        self.totalReward = 0
        self.children = {}

    def __str__(self):
        s = []
        s.append("totalReward: %s" % (self.totalReward))
        s.append("numVisits: %d" % (self.numVisits))
        s.append("isTerminal: %s" % (self.isTerminal))
        s.append("possibleActions: %s" % (self.children.keys()))
        return "%s: {%s}" % (self.__class__.__name__, ', '.join(s))


class mcts():
    def __init__(self,
                 timeLimit=None,
                 iterationLimit=None,
                 explorationConstant=1 / math.sqrt(2),
                 rolloutPolicy=randomPolicy):

        if timeLimit != None:
            if iterationLimit != None:
                raise ValueError("Cannot have both a time limit and an iteration limit")
            # time taken for each MCTS search in milliseconds
            self.timeLimit = timeLimit
            self.limitType = 'time'
        else:
            if iterationLimit == None:
                raise ValueError("Must have either a time limit or an iteration limit")
            # number of iterations of the search
            if iterationLimit < 1:
                raise ValueError("Iteration limit must be greater than one")
            self.searchLimit = iterationLimit
            self.limitType = 'iterations'

        # NOTE: c is a tuning parameter. if the reward is bounded with [0, 1], it should be sqrt(2)
        self.c = math.sqrt(2)
        self.min_reward = 10e6
        self.max_reward = -10e6

        self.explorationConstant = explorationConstant
        self.rollout = rolloutPolicy

    def search(self, initialState, needDetails=False):
        self.root = treeNode(initialState, None)

        if self.limitType == 'time':
            timeLimit = time.time() + self.timeLimit / 1000
            while time.time() < timeLimit:
                self.executeRound()
        else:
            for i in range(self.searchLimit):
                self.executeRound()

        bestChild = self.getBestChild(self.root, 0)
        action = (action for action, node in self.root.children.items() if node is bestChild).__next__()
        if needDetails:
            return {"action": action, "expectedReward": bestChild.totalReward / bestChild.numVisits}
        else:
            return action

    def executeRound(self):
        """
            execute a selection-expansion-simulation-backpropagation round
        """
        node = self.selectNode(self.root)
        reward = self.rollout(node.state)
        self.backpropogate(node, reward)

        # update c value
        if reward > self.max_reward:
            self.max_reward = reward
        if reward < self.min_reward:
            self.min_reward = reward

        self.c = max(1, self.max_reward-self.min_reward) * math.sqrt(2)

    def selectNode(self, node):
        while not node.isTerminal:
            if node.isFullyExpanded:
                node = self.getBestChild(node, self.explorationConstant)
            else:
                return self.expand(node)
        return node

    def expand(self, node):
        actions = node.state.getPossibleActions()
        for action in actions:
            if action not in node.children:
                newNode = treeNode(node.state.takeAction(action), node)
                node.children[action] = newNode
                if len(actions) == len(node.children):
                    node.isFullyExpanded = True
                return newNode

        raise Exception("Should never reach here")

    def backpropogate(self, node, reward):
        while node is not None:
            node.numVisits += 1
            node.totalReward += reward
            node = node.parent

    def getBestChild(self, node, explorationValue):
        bestValue = float("-inf")
        bestNodes = []
        for child in node.children.values():
            nodeValue = node.state.getCurrentPlayer() * child.totalReward / child.numVisits + explorationValue * self.c * \
                        math.sqrt(math.log(node.numVisits) / child.numVisits)

            if nodeValue > bestValue:
                bestValue = nodeValue
                bestNodes = [child]
            elif nodeValue == bestValue:
                bestNodes.append(child)
        return random.choice(bestNodes)
