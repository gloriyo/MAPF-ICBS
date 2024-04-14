#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs_basic import CBSSolver # original cbs with standard/disjoint splitting

# cbs with different improvements
from icbs_cardinal_bypass import ICBS_CB_Solver # only cardinal dectection and bypass
from icbs_complete import ICBS_Solver # all improvements including MA-CBS


from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost

HLSOLVER = "CBS"

LLSOLVER = "a_star"

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--hlsolver', type=str, default=HLSOLVER,
                        help='The solver to use (one of: {CBS,ICBS_CB,ICBS}), defaults to ' + str(HLSOLVER))
    # parser.add_argument('--llsolver', type=str, default=LLSOLVER,
    #                     help='The solver to use (one of: {a_star,pea_star,epea_star}), defaults to ' + str(LLSOLVER))
    args = parser.parse_args()


    result_file = open("results.csv", "w", buffering=1)

    # node_results_file = open("nodes-cleaned.csv", "w", buffering=1)

    nodes_gen_file = open("nodes-gen-cleaned.csv", "w", buffering=1)
    nodes_exp_file = open("nodes-exp-cleaned.csv", "w", buffering=1)


    if args.batch:
        
        input_instance = sorted(glob.glob("instances/test*"))
    else:
        input_instance = sorted(glob.glob(args.instance))

    for file in input_instance:

        print("***Import an instance***")

        
        print(file)
        my_map, starts, goals = import_mapf_instance(file)
        print_mapf_instance(my_map, starts, goals)

        if args.hlsolver == "CBS":
            print("***Run CBS***")
            cbs = CBSSolver(my_map, starts, goals)
            # solution = cbs.find_solution(args.disjoint)

            # if solution is not None:
            #     # print(solution)
            #     paths, nodes_gen, nodes_exp = [solution[i] for i in range(3)]
            #     if paths is None:
            #         raise BaseException('No solutions')  
            # else:
            #     raise BaseException('No solutions')

        elif args.hlsolver == "ICBS_CB":
            print("***Run ICBS with CB***")
            cbs = ICBS_CB_Solver(my_map, starts, goals)
 

        elif args.hlsolver == "ICBS":
            print("***Run ICBS***")
            cbs = ICBS_Solver(my_map, starts, goals)
            # solution = cbs.find_solution(args.disjoint)

            # if solution is not None:
            #     # print(solution)
            #     paths, nodes_gen, nodes_exp = [solution[i] for i in range(3)]
            #     if paths is None:
            #         raise BaseException('No solutions')  
            # else:
            #     raise BaseException('No solutions')



        # elif args.solver == "Independent":
        #     print("***Run Independent***")
        #     solver = IndependentSolver(my_map, starts, goals)
        #     paths, nodes_gen, nodes_exp = solver.find_solution()
        # elif args.solver == "Prioritized":
        #     print("***Run Prioritized***")
        #     solver = PrioritizedPlanningSolver(my_map, starts, goals)
        #     paths, nodes_gen, nodes_exp = solver.find_solution()

        else:
            raise RuntimeError("Unknown solver!")



        solution = cbs.find_solution(args.disjoint)

        if solution is not None:
            # print(solution)
            paths, nodes_gen, nodes_exp = [solution[i] for i in range(3)]
            if paths is None:
                raise BaseException('No solutions')  
        else:
            raise BaseException('No solutions')

        cost = get_sum_of_cost(paths)
        result_file.write("{},{}\n".format(file, cost))

        nodes_gen_file.write("{},{}\n".format(file, nodes_gen))
        nodes_exp_file.write("{},{}\n".format(file, nodes_exp))



        if not args.batch:
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            # animation.save("output.mp4", 1.0)
            animation.show()
            # animation.save('demo/fig.gif', 1)

    result_file.close()
