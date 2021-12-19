import heapq
from itertools import product
import numpy

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        # print(path)
        rst += len(path) - 1
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


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    table =dict()
    for constraint in constraints:
        print(constraints,'     ',agent)
        if constraint['agent'] == agent:
            if constraint['timestep'] not in table:
                table[constraint['timestep']] = [constraint]
            else:
                table[constraint['timestep']].append(constraint)
        # task 4
        if constraint['agent'] != agent and constraint['positive'] ==True:
            if len(constraint['loc'])>1:
                cons_i = {'agent':agent,
                            'loc':[constraint['loc'][1],constraint['loc'][0]],
                            'timestep':constraint['timestep'],
                            'positive':False
                            }
          
            else:
                cons_i = {'agent':agent,
                        'loc':constraint['loc'],
                        'timestep':constraint['timestep'],
                        'positive':False
                        }
            if cons_i['timestep'] not in table:
                table[cons_i['timestep']] = [cons_i]
            else:
                table[cons_i['timestep']].append(cons_i)

    return table
    # pass


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
    # if len(path) ==1:
    #     path = path[0]

    assert path is not None
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    if next_time in constraint_table:  
        for constraint in constraint_table[next_time]:
            if len(constraint['loc']) ==1:
                if constraint['loc'] == [next_loc]:
                    if constraint['positive']==True:
                        return 1
                    else:
                        return 0
            else:
                if constraint['loc'] ==[curr_loc,next_loc]:
                    if constraint['positive']==True:
                        return 1
                    else:
                        return 0
    return -1   
    pass


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))

def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def ma_star(my_map, start_locs, goal_loc, h_values, meta_agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - list of start position
        goal_loc    - list of goal position
        agent       - the agent that is being re-planned list of agent
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.
    
    # This is so I know which part is not CBS when debugging CBS with A*
    print("\nSTARTING A*\n") # comment out if needed

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = 0
    table = None
    start_loc = []
   

    print('# of agents', len(meta_agent))
    for agent in meta_agent:
        print('build agent {}\'s table:' )
        new_table = build_constraint_table(constraints, agent)
        if table ==None:
            table = new_table
        else:
            for key in new_table.keys():
                if key in table:
                    table[key] += new_table[key]
                else:
                    table[key] = new_table[key]
        h_value += h_values[agent][start_locs[agent]]
        start_loc.append(start_locs[agent])
    print ('table  :' ,table)
    root = {'loc': start_loc,
            'g_val': 0, 
            'h_val': h_value, 
            'parent': None,
            'timestep':0,
            'cost_after_finish':[-1 for i in range(len(meta_agent))]
            }
    push_node(open_list, root)
    closed_list[(tuple(root['loc']),root['timestep'])] = root

    
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        for i in range(len(meta_agent)):
            if curr['loc'][i] == goal_loc[meta_agent[i]] or curr['loc'][i] == goal_loc[0]:
            if curr['loc'][i] != goal_loc[meta_agent[i]]:
                break
        else: # all agents reached goal_locations
        
            print(get_path(curr,meta_agent))
            
            print("\nEND OF A*\n") # comment out if needed
            
            return get_path(curr,meta_agent)
        # # Task 1.4: Adjust the goal test condition to handle goal constraints
        
        # for i in range(len(meta_agent)):
        #     if curr['loc'][i] == goal_loc[meta_agent[i]]:
        #         no_future_goalConstraint = True
        #         for timestep in table:
        
        #             if timestep >curr['timestep']:
        #                 for cons in table[timestep]:
        #                     if cons['loc'][i] == [goal_loc] and cons['positive']==False:
        #                         no_future_goalConstraint =True
        #         if no_future_goalConstraint:
        #             return get_path(curr,meta_agent)

        meta_node = product(list(range(5)),repeat =len(meta_agent))
        for node in meta_node:
            child_loc = [move(curr['loc'][i], node[i]) for i in range(len(meta_agent))]

            continue_flag = False
            for i in range(len(child_loc)):
                # print(curr['loc'][i], '   ',child_loc[i],'    ',curr['timestep']+1)
                loc= child_loc[i]
                if loc[0]<0 or loc[0]>= len(my_map) or loc[1]<0 or loc[1]>=len(my_map[0]):
                    continue_flag = True
                    break
                if my_map[loc[0]][loc[1]]:
                    continue_flag = True
                    break
                # print('is constrainted   ',is_constrained(curr['loc'][i],loc,curr['timestep']+1,table))
                if is_constrained(curr['loc'][i],loc,curr['timestep']+1,table)==0:
                    continue_flag = True
                    break
            if continue_flag:
                continue
            
            print('agent',agent, '       ' ,child_loc)
            h_value = 0
            for i in range(len(meta_agent)):
                h_value += h_values[meta_agent[i]][child_loc[i]]

            child = {'loc': child_loc,
                    'g_val': curr['g_val'] ,
                    'h_val': h_value,
                    'parent': curr,
                    'timestep':curr['timestep']+1,
                    'cost_after_finish':curr['cost_after_finish'][:]
                    }         
            for i in range(len(meta_agent)):
                if child['loc'][i] != goal_loc[meta_agent[i]] and child['cost_after_finish'][i] != -1:                    
                    # add the missed cost to g_val
                    child['g_val'] = child['g_val'] + child['timestep'] - child['cost_after_finish'][i]                   
                    child['cost_after_finish'][i] = -1

                else:
                    if curr['loc'][i] == goal_loc[meta_agent[i]]:
                        continue
                    if child['loc'][i] == goal_loc[meta_agent[i]]:
                        if child['cost_after_finish'][i] == -1:
                            child['cost_after_finish'][i] = child['timestep']
                    child['g_val'] += 1

            if (tuple(child['loc']),child['timestep']) in closed_list:
                existing_node = closed_list[(tuple(child['loc']),child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(tuple(child['loc']),child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(tuple(child['loc']),child['timestep'])] = child
                push_node(open_list, child)   
    print('no solution')

    print("\nEND OF A*\n") # comment out if needed
    return None


