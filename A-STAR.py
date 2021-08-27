import copy
import time
import itertools


class Table:
    def __init__(self, table: list):
        self.rowSize = len(table)
        self.colSize = len(table[0])
        self.butters = []
        self.robot = []
        self.persons = []
        self.blocks = []
        self.costs = []
        i = 0
        for r in table:
            self.costs.append([])
            j = 0
            for element in r:
                if element[0] == 'x':
                    self.blocks.append([i, j])
                    self.costs[i].append(-1)
                else:
                    self.costs[i].append(int(element[0]))
                    if len(element) > 1:
                        if element[1] == 'b':
                            self.butters.append([i, j])
                        if element[1] == 'r':
                            self.robot = [i, j]
                        if element[1] == 'p':
                            self.persons.append([i, j])
                j += 1
            i += 1

    def __eq__(self, other):
        if not isinstance(other, Table):
            return NotImplemented
        butter_eq = True
        for butter in self.butters:
            if other.butters.count(butter) == 0:
                butter_eq = False
        person_eq = True
        for person in self.persons:
            if other.persons.count(person) == 0:
                person_eq = False
        block_eq = True
        for block in self.blocks:
            if other.blocks.count(block) == 0:
                block_eq = False
        return self.rowSize == other.rowSize and self.colSize == other.colSize \
            and butter_eq and self.robot == other.robot \
            and person_eq and block_eq and self.costs == other.costs


class Node:
    def __init__(self, state: Table, parent=None, last_action: str = None):
        self.state = state
        self.parent = parent
        self.last_action = last_action
        if parent:
            self.g = parent.g + self.state.costs[self.state.robot[0]][self.state.robot[1]]
            self.depth = parent.depth + 1
        else:
            self.g = 0
            self.depth = 0

        robot_butter = []
        for butter in self.state.butters:
            robot_butter.append(abs(self.state.robot[0]-butter[0])+abs(self.state.robot[1]-butter[1]))
        butter_person = manhattan_distance(self)
        self.h = min(robot_butter) - 1 + butter_person
        self.f = self.g + self.h


frontier = []
explored = []
deepest_depth = -1
goal_depth = -1


def manhattan_distance(node: Node):
    temp_table = copy.deepcopy(node.state)
    perm_butters = list(itertools.permutations(temp_table.butters))
    sum_dis = []
    for perm in perm_butters:
        temp_dis = 0
        for i in range(0, len(perm)):
            temp_dis += abs(perm[i][0]-temp_table.persons[i][0])+abs(perm[i][1]-temp_table.persons[i][1])
        sum_dis.append(temp_dis)
    return min(sum_dis)


def isInPath(state: Table):
    global frontier
    global explored
    for node in explored:
        if node.state == state:
            return True
    for node in frontier:
        if node.state == state:
            return True
    return False


def canDoIt(node: Node, action: str):
    if action == "left":
        left = node.state.robot[1]-1
        if left >= 0 and node.state.blocks.count([node.state.robot[0], left]) == 0:
            return True
    if action == "down":
        down = node.state.robot[0]+1
        if down < node.state.rowSize and node.state.blocks.count([down, node.state.robot[1]]) == 0:
            return True
    if action == "right":
        right = node.state.robot[1]+1
        if right < node.state.colSize and node.state.blocks.count([node.state.robot[0], right]) == 0:
            return True
    if action == "up":
        up = node.state.robot[0]-1
        if up >= 0 and node.state.blocks.count([up, node.state.robot[1]]) == 0:
            return True
    return False


def checkButters(node: Node, action: str):
    new_butters = copy.deepcopy(node.state.butters)
    check_node = copy.deepcopy(node)
    if action == "left":
        if check_node.state.butters.count([check_node.state.robot[0], check_node.state.robot[1]-1]):
            check_node.state.robot[1] -= 1
            if not canDoIt(check_node, "left") \
                    or check_node.state.butters.count([check_node.state.robot[0], check_node.state.robot[1]-1]):
                return False, new_butters
            new_butters.remove([check_node.state.robot[0], check_node.state.robot[1]])
            new_butters.append([check_node.state.robot[0], check_node.state.robot[1]-1])
        return True, new_butters
    if action == "down":
        if check_node.state.butters.count([check_node.state.robot[0]+1, check_node.state.robot[1]]):
            check_node.state.robot[0] += 1
            if not canDoIt(check_node, "down") \
                    or check_node.state.butters.count([check_node.state.robot[0]+1, check_node.state.robot[1]]):
                return False, new_butters
            new_butters.remove([check_node.state.robot[0], check_node.state.robot[1]])
            new_butters.append([check_node.state.robot[0]+1, check_node.state.robot[1]])
        return True, new_butters
    if action == "right":
        if check_node.state.butters.count([check_node.state.robot[0], check_node.state.robot[1]+1]):
            check_node.state.robot[1] += 1
            if not canDoIt(check_node, "right") \
                    or check_node.state.butters.count([check_node.state.robot[0], check_node.state.robot[1]+1]):
                return False, new_butters
            new_butters.remove([check_node.state.robot[0], check_node.state.robot[1]])
            new_butters.append([check_node.state.robot[0], check_node.state.robot[1]+1])
        return True, new_butters
    if action == "up":
        if check_node.state.butters.count([check_node.state.robot[0]-1, check_node.state.robot[1]]):
            check_node.state.robot[0] -= 1
            if not canDoIt(check_node, "up") \
                    or check_node.state.butters.count([check_node.state.robot[0]-1, check_node.state.robot[1]]):
                return False, new_butters
            new_butters.remove([check_node.state.robot[0], check_node.state.robot[1]])
            new_butters.append([check_node.state.robot[0]-1, check_node.state.robot[1]])
        return True, new_butters


def expand(parent: Node):
    global frontier
    global deepest_depth
    if canDoIt(parent, "up"):
        up_table = copy.deepcopy(parent.state)
        up_table.robot[0] -= 1
        flag, up_table.butters = checkButters(parent, "up")
        if flag and not isInPath(up_table):
            up_node = Node(up_table, parent, "U")
            frontier.append(up_node)
            if up_node.depth > deepest_depth:
                deepest_depth = up_node.depth
    if canDoIt(parent, "right"):
        right_table = copy.deepcopy(parent.state)
        right_table.robot[1] += 1
        flag, right_table.butters = checkButters(parent, "right")
        if flag and not isInPath(right_table):
            right_node = Node(right_table, parent, "R")
            frontier.append(right_node)
            if right_node.depth > deepest_depth:
                deepest_depth = right_node.depth
    if canDoIt(parent, "down"):
        down_table = copy.deepcopy(parent.state)
        down_table.robot[0] += 1
        flag, down_table.butters = checkButters(parent, "down")
        if flag and not isInPath(down_table):
            down_node = Node(down_table, parent, "D")
            frontier.append(down_node)
            if down_node.depth > deepest_depth:
                deepest_depth = down_node.depth
    if canDoIt(parent, "left"):
        left_table = copy.deepcopy(parent.state)
        left_table.robot[1] -= 1
        flag, left_table.butters = checkButters(parent, "left")
        if flag and not isInPath(left_table):
            left_node = Node(left_table, parent, "L")
            frontier.append(left_node)
            if left_node.depth > deepest_depth:
                deepest_depth = left_node.depth


def goalTest(node: Node):
    global goal_depth
    for butter in node.state.butters:
        if node.state.persons.count(butter) == 0:
            return False
    goal_depth = node.depth
    return True


def write_path(goal, write):
    if goal:
        write_path(goal.parent, write)
        if goal.last_action:
            write.write(goal.last_action + " ")


def min_cost_finder():
    global frontier
    global explored
    best = frontier[0]
    for node in frontier:
        if node.f < best.f:
            best = node
    frontier.remove(best)
    explored.append(best)
    return best


def A_star(init: Table, cutoff: int, write):
    global frontier
    global explored
    init_node = Node(copy.deepcopy(init))
    frontier.append(init_node)
    while len(frontier):
        pop = min_cost_finder()
        if goalTest(pop):
            write_path(pop, write)
            write.write("\n" + str(pop.f) + "\n" + str(pop.depth))
            return True
        if pop.depth < cutoff:
            expand(pop)
    write.write("can’t pass the butter")
    return False


if __name__ == '__main__':
    test_read = open("input/test3.txt", "r")
    dimension = list(map(int, test_read.readline().split()))
    init_table_list = []
    for row in range(0, dimension[0]):
        init_table_list.append(list(test_read.readline().split()))
    test_read.close()

    init_table = Table(init_table_list)
    test_write = open("output/A-STAR_output/output3.txt", "w")
    startTime = time.time()
    A_star(init_table, 50, test_write)
    endTime = time.time()
    print("\nexec time:", int((endTime - startTime) / 60), "minutes and",
          "{:.2f}".format((endTime - startTime) % 60), "seconds")
    print("#generated_nodes:", len(frontier) + len(explored), ", #expanded_nodes:", len(explored))
    print("depth of deepest node:", deepest_depth, ", depth of goal:", goal_depth)
    test_write.close()
