import heapq
from itertools import product
import numpy as np
import copy

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        print(path)
        rst += len(path) - 1
        if(len(path)>1):
            assert path[-1] != path[-2]
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values

# # return a table that constains the list of constraints of all agents for each time step. 
# def build_constraint_table(constraints, meta_agent):
#     # constraint_table = {}
#     constraint_table = collections.defaultdict(list) # dictionary of lists

#     if not constraints:
#         return constraint_table
#     for constraint in constraints:
#         timestep = constraint['timestep']
#         for agent in meta_agent:
#             # positive and negative constraint for agent
#             if (constraint['agent'] == agent):
#                 constraint_table[timestep].append(constraint)
#             # enforce positive constraints from other agents (i.e. create neg constraint)
#             elif constraint['positive']: 
#                 neg_constraint = copy.deepcopy(constraint)
#                 neg_constraint['agent'] = agent
#                 neg_constraint['meta_agent'] = meta_agent
#                 # if edge collision
#                 if len(constraint['loc']) == 2:
#                     # switch traversal direction
#                     prev_loc = constraint['loc'][1]
#                     curr_loc = constraint['loc'][0]
#                     neg_constraint['loc'] = [prev_loc, curr_loc]
#                 neg_constraint['positive'] = False
#                 constraint_table[timestep].append(neg_constraint)
    
#     return constraint_table
                
# return a table that constains the list of constraints of a given agent for each time step. 
def build_constraint_table(constraints, agent):
    # constraint_table = {}
    # constraint_table = [[] for i in range(3)]
    constraint_table = dict()

    if not constraints:
        return constraint_table
    for constraint in constraints:

        # print(constraint)

        timestep = constraint['timestep']

        t_constraint = []
        if timestep in constraint_table:
            t_constraint = constraint_table[timestep]

        # positive constraint for agent
        if constraint['positive'] and constraint['agent'] == agent:
            
            # constraint_table[timestep].append(constraint)
            t_constraint.append(constraint)
            constraint_table[timestep] = t_constraint
        # and negative (external) constraint for agent
        elif not constraint['positive'] and (constraint['agent'] == agent or agent in constraint['meta_agent']):
            # constraint_table[timestep].append(constraint)
            t_constraint.append(constraint)
            constraint_table[timestep] = t_constraint
        # enforce positive constraints from other agents (i.e. create neg constraint)
        elif constraint['positive']: 
            assert not (constraint['agent'] == agent)
            neg_constraint = copy.deepcopy(constraint)
            neg_constraint['agent'] = agent
            # neg_constraint['meta_agent'] = meta_agent
            # if edge collision
            if len(constraint['loc']) == 2:
                # switch traversal direction
                prev_loc = constraint['loc'][1]
                curr_loc = constraint['loc'][0]
                neg_constraint['loc'] = [prev_loc, curr_loc]
            neg_constraint['positive'] = False
            # constraint_table[timestep].append(neg_constraint)
            t_constraint.append(neg_constraint)
            constraint_table[timestep] = t_constraint
    
    return constraint_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node,meta_agent):
    path = []
    for i in range(len(meta_agent)):
        path.append([])
    curr = goal_node
    while curr is not None:
        for i in range(len(meta_agent)):
            path[i].append(curr['loc'][i])
        curr = curr['parent']
    for i in range(len(meta_agent)):
        path[i].reverse()
        assert path[i] is not None

        # print(path[i])

        if len(path[i]) > 1: 
            # remove trailing duplicates
            while path[i][-1] == path[i][-2]:
                path[i].remove(path[i][-1])
                if len(path[i]) <= 1:
                    break
            # assert path[i][-1] != path[i][-2] # no repeats at the end!!

    assert path is not None
    return path

# returns whether if a move at timestep violates any external (negative) constraint
def is_constrained(curr_loc, next_loc, timestep, constraint_table, agent):

    # print("the move : {}, {}".format(curr_loc, next_loc))

    if timestep not in constraint_table:
        return False
    
    for constraint in constraint_table[timestep]:
        if agent == constraint['agent'] and constraint['positive'] == False:
            # vertex constraint
            if len(constraint['loc']) == 1:
                if next_loc == constraint['loc'][0]:
                    print("the move : {}, {}  time {}".format(curr_loc, next_loc,timestep))
                    print("vertex constraint", constraint)
                    return True
            # edge constraint
            else:
                if constraint['loc'] == [curr_loc, next_loc]:
                    print("the move : {}, {}  time {}".format(curr_loc, next_loc,timestep))
                    print("edge constraint", constraint)
                    return True

    return False

# returns whether agent violates its positive constraint
def violates_pos_constraint(curr_loc, next_loc, timestep, constraint_table, agent, meta_agent):
    if timestep not in constraint_table:
        return False
    for constraint in constraint_table[timestep]:
        if agent == constraint['agent'] and constraint['positive']:
            # vertex constraint
            if len(constraint['loc']) == 1:
                if next_loc != constraint['loc'][0]:
                    # print('agent {} must follow positive constraint at timestep {}: {}'.format(agent, timestep, constraint['loc']))
                    return True
            # edge constraint
            else:
                if constraint['loc'] != [curr_loc, next_loc]:
                    # print('agent {} must follow positive constraint at timestep {}: {}'.format(agent, timestep, constraint['loc']))
                    return True
    return False
    



def push_node(open_list, node):
    if not isinstance(node['g_val'] + node['h_val'], int):
        # print(node['g_val'] + node['h_val'])
        assert False
    f_value = node['g_val'] + node['h_val']

    # node_is_in_open_list = any(node == open_node[4] for open_node in open_list)

    # for open_node in open_list:
    #     print(node, open_node[4])

    # print(node_is_in_open_list)
    # if not node_is_in_open_list:
    

    heapq.heappush(open_list, (node['F_val'], f_value, node['h_val'], node['loc'], node['i_node'], node))

def pop_node(open_list):
    # if len(open_list) > 1:
    #     print("1\n")
    #     print(open_list[0])
    #     print("2\n")
    #     print(open_list[1])    
    _, _, _, _, id, curr = heapq.heappop(open_list)
    print("> Expand node {} with F_val {}".format(id, curr['F_val']))
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def generate_child_nodes(my_map, h_values, goal_loc, max_constraints, constraint_table, curr, meta_agent, node_i):
    children = []
    ma_dirs = product(list(range(5)), repeat=len(meta_agent)) # directions for move() for each agent: 0, 1, 2, 3, 4
    
    for dirs in ma_dirs: 
        # print(dirs)
        invalid_move = False
        child_loc = []
        # move each agent for new timestep & check for (internal) conflicts with each other
        for i, a in enumerate(meta_agent):           
                aloc = move(curr['loc'][i], dirs[i])
                # vertex collision; check for duplicates in child_loc
                if aloc in child_loc:
                    invalid_move = True
                    # print("internal conflict")
                    break
                child_loc.append(move(curr['loc'][i], dirs[i]))   


        if invalid_move:
            continue


        for i, a in enumerate(meta_agent):   
            # edge collision: check for matching locs in curr_loc and child_loc between two agents
            for j, a in enumerate(meta_agent):   
                if i != j:
                    # print(ai, aj)
                    if child_loc[i] == curr['loc'][j] and child_loc[j] == curr['loc'][i]:
                        invalid_move = True             
        
        if invalid_move:
            continue

        # check map constraints and external constraints
        for i, a in enumerate(meta_agent):  
            loc= child_loc[i]
            # agent out of map bounds
            if loc[0]<0 or loc[0]>=len(my_map) or loc[1]<0 or loc[1]>=len(my_map[0]):
                invalid_move = True
            # agent collison with map obstacle
            elif my_map[loc[0]][loc[1]]:
                invalid_move = True
            # agent is constrained by a negative external constraint
            elif is_constrained(curr['loc'][i],loc,curr['timestep']+1,constraint_table[i], meta_agent[i]):
                # print()
                print("AGENT {} IS CONSTRAINED AT T {}; CONTINUE".format(a, curr['timestep']+1))
                invalid_move = True
            # # agent has a positive constraint and doesn't meet its positive constraint
            # elif violates_pos_constraint(curr['loc'][i],loc,curr['timestep']+1,constraint_table[i], meta_agent[i], meta_agent):
            #     print("currently at timestep {} with child loc {}".format(curr['timestep'], loc))
            #     invalid_move = True
            if invalid_move:
                break

        if invalid_move:
            continue

        # find h_values for current moves
        h_value = 0
        for i, a in enumerate(meta_agent):  
            h_value += h_values[meta_agent[i]][child_loc[i]]

        # g_value = curr['g_val']+ curr['reached_goal'].count(False)
        num_moves = curr['reached_goal'].count(False)
        g_value = curr['g_val'] + num_moves


        reached_goal = [False for i in range(len(meta_agent))]
        for i, a in enumerate(meta_agent):
            # print(child_loc[i], goal_loc[a])
            # print(max_constraints[i], curr['timestep']+1)
            
            if child_loc[i] == goal_loc[a] and (curr['timestep']+1 > max_constraints[i]):
                print("agent ", a, 'has reached_goal at timestep ', curr['timestep'] + 1)
                print (max_constraints[i])
                reached_goal[i] = True



        child = {'i_node': node_i,
                'loc': child_loc,
                'F_val': g_value+h_value,
                'g_val': g_value, # number of new locs (cost) added
                'h_val': h_value,
                'parent': curr,
                'timestep': curr['timestep']+1,
                'reached_goal': copy.deepcopy(reached_goal)
                }

        node_i += 1        


        # print(child)

        children.append(child)
        
        # num_node_generated += 1

    return children, node_i


# Partial Expansion A* which generates but does not add surplus nodes to the open list
def pea_star(my_map, start_locs, goal_loc, h_values, meta_agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - list of start position
        goal_loc    - list of goal position
        agent       - the agent that is being re-planned list of agent
        constraints - constraints defining where agent must or cannot go at each timestep
    """

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = 0
    table = None

   # check if meta_agent is only a simple agent (from basic CBS)
    if not isinstance(meta_agent, list):
        meta_agent = [meta_agent]
        # print(meta_agent)

        # add meta_agent keys to constraints
        for c in constraints:
            c['meta_agent'] = {c['agent']}

    num_node_generated = 0

    ma_length = len(meta_agent)

    table = []
    max_constraints = np.zeros((ma_length,), dtype=int)

    
    for i, a in enumerate(meta_agent):
        table_i = build_constraint_table(constraints, a)
        table.append(table_i)
        if table_i.keys():
            max_constraints[i] = max(table_i.keys())

    print (table)

    # print("> build constraint table")
    # print(table)
    # print(max_constraints)

    # combined h value for agents in meta-agent
    for agent in meta_agent:
        # print(agent)
        h_value += h_values[agent][start_locs[agent]]

    root = {'i_node': num_node_generated,
            'loc': [start_locs[a] for a in meta_agent],
            'F_val' : h_value, # only consider children with f_val == F_val
            'g_val': 0, 
            'h_val': h_value, 
            'parent': None,
            'timestep': 0,
            'reached_goal': [False for i in range(len(meta_agent))]
            }

    num_node_generated += 1

    push_node(open_list, root)
    # closed_list[(tuple(root['loc']),root['timestep'])] = root

    
    while len(open_list) > 0:

        # if num_node_generated >= 30:
        #     return

        curr = pop_node(open_list)

        # print("t {}, curr loc {}".format(curr['timestep'], curr['loc']))

        # # check if any agent is at their goal loc (shouldn't move in child loc)
        # for i, a in enumerate(meta_agent):

        #     # print('\'agent\': {}, \'timestep\': {}'.format(a, curr['timestep']))

        #     if curr['loc'][i] == goal_loc[a] and curr['timestep'] >= max_constraints[i]:
        #         break

        # else: # all agents reached goal_locations
        
        #     print('Returning path....')

        #     return get_path(curr,meta_agent)

        solution_found = all(curr['reached_goal'][i] for i in range(len(meta_agent)))
        # print(curr['reached_goal'] )

        if solution_found:
            return get_path(curr,meta_agent)

        if(len(max_constraints) == 2):
            if curr['timestep'] > max_constraints[0] and curr['timestep'] > max_constraints[1]:
                # print(curr['loc'][0] == goal_loc[meta_agent[0]], curr['loc'][1] == goal_loc[meta_agent[1]])
                assert not(curr['loc'][0] == goal_loc[meta_agent[0]] and curr['loc'][1] == goal_loc[meta_agent[1]])

        # ma_dirs_list = []

        # seeking_ma = copy.deepcopy(meta_agent)
        # # remove agent that has reached its goal from ma


        # num_a_path_complete = 0
        # for i, a in enumerate(meta_agent):
        #     if curr['reached_goal'][i] == True:
        #         seeking_ma.remove(a)
        #         num_a_path_complete += 1

        # s_ma_length = len(seeking_ma)

        # assert len(seeking_ma) == ma_length - num_a_path_complete

        # # create a list of lists of each possible directions for remaining agents
        # for a in range(s_ma_length):
        #     ma_dirs_list.append(list(range(5)))

        # # # create a list of lists of each possible directions for each agent 
        # # for a in range(ma_length):
        # #     if curr['reached_goal'][a] == True:
        # #         ma_dirs_list.append([4]) # do NOT move agent
        # #         # print('agent {} has reached goal: {} at t {}; do not move'.format(meta_agent[a], curr['loc'][a], curr['timestep']))
        # #     # elif : # idk how to deal with this yet... check curr loc to see if
        # #     #     is_pos_constrained(curr['timestep']+1,table, meta_agent[a])
        # #     else:
        # #         ma_dirs_list.append(list(range(5)))

        # # all combinations of directions for each agent in meta_agent for next timestep
        # # ma_dirs = product(list(range(5)),repeat =len(meta_agent))
        
        # ma_dirs = product(*ma_dirs_list) # create "nested loop with available moves"

       
        # # generate children of curr


        children, num_node_generated = generate_child_nodes(my_map, h_values, goal_loc, max_constraints, table, curr, meta_agent, num_node_generated)

        # num_node_generated += ma_length ** 5

        next_best_f = False



        for child in children:


            # print("curr F: ", curr['F_val'])
            # print("child f: ", child['g_val'] + child['h_val'])
            # print(child['reached_goal'])
            # assert child_loc[i] != goal_loc[a]
            # assert child['reached_goal'] != [True]

            f_value = child['g_val'] + child['h_val']

            # add children if it's f_val is equal to curr's
            if f_value == curr['F_val']:
                # print("curr F: ", curr['F_val'])
                # print("child f: ", child['g_val'] + child['h_val'])
                if (tuple(child['loc']),child['timestep']) not in closed_list:
                    # existing_node = closed_list[(tuple(child['loc']),child['timestep'])]
                    # if compare_nodes(child, existing_node):
                    closed_list[(tuple(child['loc']),child['timestep'])] = child
                    # print('bye child ',child['loc'])
                    push_node(open_list, child)
                # else:
                #     # print(closed_list[(tuple(child['loc']),child['timestep'])])
                #     assert child['reached_goal'] != [True]
                # else:
                #     closed_list[(tuple(child['loc']),child['timestep'])] = child
                    # push_node(open_list, child) 
                
            elif f_value > curr['F_val']:
                # print("curr F: ", curr['F_val'])
                # print("child f: ", child['g_val'] + child['h_val'])
                next_best_f = f_value if not next_best_f else min(next_best_f, f_value)

        # push curr back into open list with best f_val > curr['f_val']
        if next_best_f:
            curr['F_val'] = next_best_f
            # curr['g_val'] = next_best_f['g_val']
            # curr['h_val'] = next_best_f['h_val']
            push_node(open_list, curr)
        # if curr has no unexpanded children, add to closed list 
        elif (tuple(curr['loc']),curr['timestep']) not in closed_list:
            # print('die parent', curr['loc'])
            closed_list[(tuple(curr['loc']),curr['timestep'])] = curr

    print('no solution')

    # print("\nEND OF A*\n") # comment out if needed
    return None


