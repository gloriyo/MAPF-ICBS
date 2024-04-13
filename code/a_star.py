import heapq
from itertools import product
from typing import List
import numpy
import copy
import collections

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
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
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values

# return a table that constains the list of constraints of all agents for each time step. 
def build_constraint_table(constraints, meta_agent):
    # constraint_table = {}
    constraint_table = collections.defaultdict(list) # dictionary of lists

    if not constraints:
        return constraint_table
    for constraint in constraints:
        timestep = constraint['timestep']
        for agent in meta_agent:
            # positive and negative constraint for agent
            if (constraint['agent'] == agent):
                constraint_table[timestep].append(constraint)
            # enforce positive constraints from other agents (i.e. create neg constraint)
            elif constraint['positive']: 
                neg_constraint = copy.deepcopy(constraint)
                neg_constraint['agent'] = agent
                neg_constraint['meta_agent'] = meta_agent
                # if edge collision
                if len(constraint['loc']) == 2:
                    # switch traversal direction
                    prev_loc = constraint['loc'][1]
                    curr_loc = constraint['loc'][0]
                    neg_constraint['loc'] = [prev_loc, curr_loc]
                neg_constraint['positive'] = False
                constraint_table[timestep].append(neg_constraint)
    
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

        print(path[i])

        if len(path[i]) > 1: 
            # remove trailing duplicates
            while path[i][-1] == path[i][-2]:
                path[i].remove(path[i][-1])
            assert path[i][-1] != path[i][-2] # no repeats at the end!!

    assert path is not None
    return path

# returns whether if a move at timestep violates any external (negative) constraint
def is_constrained(curr_loc, next_loc, timestep, constraint_table, agent):

    if timestep not in constraint_table:
        return False
    
    for constraint in constraint_table[timestep]:
        if agent == constraint['agent'] and constraint['positive'] == False:
            # vertex constraint
            if len(constraint['loc']) == 1:
                if next_loc == constraint['loc'][0]:
                    return True
            # edge constraint
            else:
                if constraint['loc'] == [curr_loc, next_loc]:
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
                    print('agent {} must follow positive constraint at timestep {}: {}'.format(agent, timestep, constraint['loc']))
                    return True
            # edge constraint
            else:
                if constraint['loc'] != [curr_loc, next_loc]:
                    print('agent {} must follow positive constraint at timestep {}: {}'.format(agent, timestep, constraint['loc']))
                    return True
    return False
    

# future external constraints
def future_constraint_exists(agent, meta_agent, agent_loc, timestep, constraint_table):
    for t in constraint_table:
        # if t is a future timestep which exists in constraint table
        if t > timestep:

            for constraint in constraint_table[t]:
                # last loc in vertex/edge constraint
                if constraint['loc'][-1] == agent_loc:

                    if agent == 2:
                        print('current timestep: {}, const timestep: {}'.format(timestep, constraint['timestep']))
                        print('agent 2 loc: ', agent_loc)
                        print('constraint loc:', constraint['loc'][-1])

                    if(agent == constraint['agent'] and not constraint['positive']):
                        return True
                    if(agent != constraint['agent'] and constraint['positive']):
                        return True
                
    return False



def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))

def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_locs, goal_loc, h_values, meta_agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - list of start positions
        goal_loc    - list of goal positions
        meta_agent  - the agent (CBS) or meta-agent of the agent (MA-CBS) involved in collision
        constraints - constraints generated by a CBS splitter; dict = {agent,loc,timestep,positive}
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    h_value = 0
    table = None

    # check if meta_agent is only a simple agent (from basic CBS)
    if not isinstance(meta_agent, list):
        meta_agent = [meta_agent]
        # add meta_agent keys to constraints
        for c in constraints:
            c['meta_agent'] = {c['agent']}
    ma_length = len(meta_agent)

    table = build_constraint_table(constraints, meta_agent)

    print("> build constraint table")
    print(table)

    # combined h value for agents in meta-agent
    for agent in meta_agent:
        h_value = h_values[agent][start_locs[agent]]

    root = {'loc': [start_locs[a] for a in meta_agent],
            'g_val': 0, 
            'h_val': h_value, 
            'parent': None,
            'timestep': 0,
            'reached_goal': [False for i in range(len(meta_agent))]
            }

    push_node(open_list, root)
    closed_list[(tuple(root['loc']),root['timestep'])] = root

    
    while len(open_list) > 0:
        curr = pop_node(open_list)

        # print('pop node w/ f val: ', curr['g_val'] + curr['h_val'])

        # check if any agent is at their goal loc (shouldn't move in child loc)
        for a in range(ma_length):

            # print('\'agent\': {}, \'timestep\': {}'.format(a, curr['timestep']))

            if curr['loc'][a] == goal_loc[meta_agent[a]]:
                # check if there are any future (external) constraints
                future_constraint_found = future_constraint_exists(meta_agent[a],meta_agent, curr['loc'][a], curr['timestep'], table)
                if future_constraint_found:
                    print("future constraint found!!")
                else:
                    curr['reached_goal'][a] = True

        # check if all agents have reached their goal states
        for a in range(len(meta_agent)):
            if curr['reached_goal'][a] == False:
                break
        else: # all agents reached goal_locations

            # else: # all agents do not violate future constraints
                print('Returning path....')
                # print(get_path(curr,meta_agent), '\n')
                
                # ALSO AGENTS WITH POSITIVE CONSTRAINTS SHOULD ABIDE BY THEM
                return get_path(curr,meta_agent)

        ma_dirs_list = []

        seeking_ma = copy.deepcopy(meta_agent)
        # remove agent that has reached its goal from ma
        num_a_path_complete = 0
        for i, a in enumerate(meta_agent):
            if curr['reached_goal'][i] == True:
                seeking_ma.remove(a)
                num_a_path_complete += 1

        s_ma_length = len(seeking_ma)

        assert len(seeking_ma) == ma_length - num_a_path_complete

        # create a list of lists of each possible directions for remaining agents
        for a in range(s_ma_length):
            ma_dirs_list.append(list(range(5)))

        # all combinations of directions for each agent in meta_agent for next timestep
        ma_dirs = product(*ma_dirs_list) # create "nested loop with available moves"

        # each 'dirs' contains 1 possible direction for each remaining agent 
        for dirs in ma_dirs:
            # print('dirs: ', dirs)

            invalid_move = False
            child_loc = copy.deepcopy(curr['loc'])
            # move each agent for new timestep & check for (internal) conflicts with each other
            for a in range(ma_length):           
                if curr['reached_goal'][a] == True:
                    agent = meta_agent[a]
                    assert agent not in seeking_ma

                    continue
                else:
                    agent = meta_agent[a]
                    i_dir = seeking_ma.index(agent) # index in directions list
                    assert i_dir >= 0
                    aloc = move(curr['loc'][a], dirs[i_dir])
                    # vertex collision; check for duplicates in child_loc
                    if aloc in child_loc:
                        invalid_move = True
                        break
                    child_loc[a] = move(curr['loc'][a], dirs[i_dir])   


            if invalid_move:
                continue


            for ai in range(ma_length):
                # edge collision: check for matching locs in curr_loc and child_loc between two agents
                for aj in range(ma_length):
                    if ai != aj:
                        if child_loc[ai] == curr['loc'][aj] and child_loc[aj] == curr['loc'][ai]:
                            invalid_move = True             
            
            if invalid_move:
                continue

            # check map constraints and external constraints
            for i in range(len(child_loc)):
                loc= child_loc[i]
                # agent out of map bounds
                if loc[0]<0 or loc[0]>= len(my_map) or loc[1]<0 or loc[1]>=len(my_map[0]):
                    invalid_move = True
                    break
                # agent collison with map obstacle
                if my_map[loc[0]][loc[1]]:
                    invalid_move = True
                    break
                # agent is constrained by a negative external constraint
                if is_constrained(curr['loc'][i],loc,curr['timestep']+1,table, meta_agent[i]):
                    invalid_move = True
                    break
                # agent has a positive constraint and doesn't meet its positive constraint
                if violates_pos_constraint(curr['loc'][i],loc,curr['timestep']+1,table, meta_agent[i], meta_agent):
                    print("currently at timestep {} with child loc {}".format(curr['timestep'], loc))
                    invalid_move = True
                    break

            if invalid_move:
                continue

            # find h_values for current moves
            h_value = 0
            for i in range(ma_length):
                h_value += h_values[meta_agent[i]][child_loc[i]]



            child = {'loc': child_loc,
                    'g_val': curr['g_val']+s_ma_length, # number of new locs (cost) added
                    'h_val': h_value,
                    'parent': curr,
                    'timestep':curr['timestep']+1,
                    'reached_goal': [False for i in range(len(meta_agent))] # currently checked when node is popped
                    }
            

            if (tuple(child['loc']),child['timestep']) in closed_list:
                existing_node = closed_list[(tuple(child['loc']),child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(tuple(child['loc']),child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(tuple(child['loc']),child['timestep'])] = child
                push_node(open_list, child)   
    print('no solution')

    return None
