import copy
import time


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
            self.depth = parent.depth + 1
        else:
            self.depth = 0


frontier1 = []
frontier2 = []
explored1 = []
explored2 = []
generated_nodes = 0
expanded_nodes = 0


def isInPath(state: Table, which: int):
    global frontier1
    global frontier2
    global explored1
    global explored2
    if which == 1:
        for node in explored2:
            if node.state == state:
                return True
        for node in frontier2:
            if node.state == state:
                return True
    elif which == 0:
        for node in explored1:
            if node.state == state:
                return True
        for node in frontier1:
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
    global frontier1
    if canDoIt(parent, "up"):
        up_table = copy.deepcopy(parent.state)
        up_table.robot[0] -= 1
        flag, up_table.butters = checkButters(parent, "up")
        if flag and not isInPath(up_table, 0):
            up_node = Node(up_table, parent, "U")
            frontier1.append(up_node)
    if canDoIt(parent, "right"):
        right_table = copy.deepcopy(parent.state)
        right_table.robot[1] += 1
        flag, right_table.butters = checkButters(parent, "right")
        if flag and not isInPath(right_table, 0):
            right_node = Node(right_table, parent, "R")
            frontier1.append(right_node)
    if canDoIt(parent, "down"):
        down_table = copy.deepcopy(parent.state)
        down_table.robot[0] += 1
        flag, down_table.butters = checkButters(parent, "down")
        if flag and not isInPath(down_table, 0):
            down_node = Node(down_table, parent, "D")
            frontier1.append(down_node)
    if canDoIt(parent, "left"):
        left_table = copy.deepcopy(parent.state)
        left_table.robot[1] -= 1
        flag, left_table.butters = checkButters(parent, "left")
        if flag and not isInPath(left_table, 0):
            left_node = Node(left_table, parent, "L")
            frontier1.append(left_node)


def pCanDoIt(node: Node, action: str):
    if action == "left":
        left = node.state.robot[1]-1
        if left >= 0 and node.state.blocks.count([node.state.robot[0], left]) == 0\
                and node.state.butters.count([node.state.robot[0], left]) == 0:
            return True
    if action == "down":
        down = node.state.robot[0]+1
        if down < node.state.rowSize and node.state.blocks.count([down, node.state.robot[1]]) == 0\
                and node.state.butters.count([down, node.state.robot[1]]) == 0:
            return True
    if action == "right":
        right = node.state.robot[1]+1
        if right < node.state.colSize and node.state.blocks.count([node.state.robot[0], right]) == 0\
                and node.state.butters.count([node.state.robot[0], right]) == 0:
            return True
    if action == "up":
        up = node.state.robot[0]-1
        if up >= 0 and node.state.blocks.count([up, node.state.robot[1]]) == 0\
                and node.state.butters.count([up, node.state.robot[1]]) == 0:
            return True
    return False


def pCheckButters(node: Node, action: str):
    if action == "left":
        if node.state.butters.count([node.state.robot[0], node.state.robot[1] - 1]):
            return True
    if action == "down":
        if node.state.butters.count([node.state.robot[0] + 1, node.state.robot[1]]):
            return True
    if action == "right":
        if node.state.butters.count([node.state.robot[0], node.state.robot[1] + 1]):
            return True
    if action == "up":
        if node.state.butters.count([node.state.robot[0] - 1, node.state.robot[1]]):
            return True
    return False


def pExpand(child: Node):
    global frontier2
    if pCanDoIt(child, "up"):
        up_table = copy.deepcopy(child.state)
        up_table.robot[0] -= 1
        if not isInPath(up_table, 1):
            up_node = Node(up_table, child, "D")
            frontier2.append(up_node)
        if pCheckButters(child, "down"):
            up_table2 = copy.deepcopy(up_table)
            up_table2.butters.remove([child.state.robot[0]+1, child.state.robot[1]])
            up_table2.butters.append([child.state.robot[0], child.state.robot[1]])
            if not isInPath(up_table2, 1):
                up_node2 = Node(up_table2, child, "D")
                frontier2.append(up_node2)
    if pCanDoIt(child, "right"):
        right_table = copy.deepcopy(child.state)
        right_table.robot[1] += 1
        if not isInPath(right_table, 1):
            right_node = Node(right_table, child, "L")
            frontier2.append(right_node)
        if pCheckButters(child, "left"):
            right_table2 = copy.deepcopy(right_table)
            right_table2.butters.remove([child.state.robot[0], child.state.robot[1]-1])
            right_table2.butters.append([child.state.robot[0], child.state.robot[1]])
            if not isInPath(right_table2, 1):
                right_node2 = Node(right_table2, child, "L")
                frontier2.append(right_node2)
    if pCanDoIt(child, "down"):
        down_table = copy.deepcopy(child.state)
        down_table.robot[0] += 1
        if not isInPath(down_table, 1):
            down_node = Node(down_table, child, "U")
            frontier2.append(down_node)
        if pCheckButters(child, "up"):
            down_table2 = copy.deepcopy(down_table)
            down_table2.butters.remove([child.state.robot[0]-1, child.state.robot[1]])
            down_table2.butters.append([child.state.robot[0], child.state.robot[1]])
            if not isInPath(down_table2, 1):
                down_node2 = Node(down_table2, child, "U")
                frontier2.append(down_node2)
    if pCanDoIt(child, "left"):
        left_table = copy.deepcopy(child.state)
        left_table.robot[1] -= 1
        if not isInPath(left_table, 1):
            left_node = Node(left_table, child, "R")
            frontier2.append(left_node)
        if pCheckButters(child, "right"):
            left_table2 = copy.deepcopy(left_table)
            left_table2.butters.remove([child.state.robot[0], child.state.robot[1]+1])
            left_table2.butters.append([child.state.robot[0], child.state.robot[1]])
            if not isInPath(left_table2, 1):
                left_node2 = Node(left_table2, child, "R")
                frontier2.append(left_node2)


def goalTest():
    global frontier1
    global explored1
    global explored2
    global frontier2
    for node1 in frontier1:
        for node2 in frontier2:
            if node1.state == node2.state:
                return True, node1, node2
    return False, None, None


def write_path(node1: Node, node2: Node, write):
    temp = copy.deepcopy(node1)
    node1_acts = []
    while temp:
        if temp.last_action:
            node1_acts.append(temp.last_action)
        temp = temp.parent
    node1_acts.reverse()
    for act in node1_acts:
        write.write(act + " ")
    temp = copy.deepcopy(node2)
    main_goal = copy.deepcopy(temp)
    while temp and temp.state == main_goal.state:
        for butter in temp.state.butters:
            if temp.state.persons.count(butter) == 0:
                if temp.last_action:
                    write.write(temp.last_action + " ")
                main_goal = copy.deepcopy(temp.parent)
                break
        temp = temp.parent
    write.write("\n"+str(node1.depth+node2.depth-main_goal.depth)+"\n"+str(node1.depth+node2.depth-main_goal.depth))


def find_goal(init: Table, robot_place: list):
    goal = copy.deepcopy(init)
    goal.butters = copy.deepcopy(goal.persons)
    goal.robot = copy.deepcopy(robot_place)
    return goal


def BDirectBFS(init: Table, cutoff: int, robot_place: list):
    global frontier1
    global frontier2
    global explored1
    global explored2
    init_node = Node(copy.deepcopy(init))
    goal = find_goal(init, robot_place)
    goal_node = Node(copy.deepcopy(goal))
    frontier1.append(init_node)
    frontier2.append(goal_node)
    while len(frontier1) or len(frontier2):
        flag, node1, node2 = goalTest()
        if flag:
            return True, node1, node2
        if len(frontier1):
            pop1 = frontier1[0]
            depth1 = pop1.depth
            while len(frontier1) and depth1 == pop1.depth:
                frontier1.pop(0)
                explored1.append(pop1)
                if pop1.depth < cutoff:
                    expand(pop1)
                if len(frontier1):
                    pop1 = frontier1[0]
        flag, node1, node2 = goalTest()
        if flag:
            return True, node1, node2
        if len(frontier2):
            pop2 = frontier2[0]
            depth2 = pop2.depth
            while len(frontier2) and depth2 == pop2.depth:
                frontier2.pop(0)
                explored2.append(pop2)
                if pop2.depth < cutoff:
                    pExpand(pop2)
                if len(frontier2):
                    pop2 = frontier2[0]
    return False, None, None


def Best_BDirectBFS(init: Table, cutoff: int, write):
    global frontier1
    global frontier2
    global explored1
    global explored2
    global generated_nodes
    global expanded_nodes
    best_node1 = None
    best_node2 = None
    best_dis = 10000
    for person in init.persons:
        flag, temp1, temp2 = BDirectBFS(init, cutoff, person)
        if flag:
            temp = copy.deepcopy(temp2)
            main_goal = copy.deepcopy(temp)
            while temp and temp.state == main_goal.state:
                for butter in temp.state.butters:
                    if temp.state.persons.count(butter) == 0:
                        main_goal = copy.deepcopy(temp.parent)
                        break
                temp = temp.parent
            temp_dis = temp1.depth+temp2.depth-main_goal.depth
            if temp_dis < best_dis:
                best_node1 = temp1
                best_node2 = temp2
                best_dis = temp_dis
        generated_nodes += len(frontier1)+len(frontier2)+len(explored1)+len(explored2)
        expanded_nodes += len(explored1)+len(explored2)
        frontier1 = []
        frontier2 = []
        explored1 = []
        explored2 = []
    if best_node1 and best_node2:
        write_path(best_node1, best_node2, write)
        return True
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
    test_write = open("output/BDirectBFS_output/output3.txt", "w")
    startTime = time.time()
    Best_BDirectBFS(init_table, 50, test_write)
    endTime = time.time()
    print("\nexec time:", int((endTime - startTime) / 60), "minutes and",
          "{:.2f}".format((endTime - startTime) % 60), "seconds")
    print("#generated_nodes:", generated_nodes, ", #expanded_nodes:", expanded_nodes)
    test_write.close()
