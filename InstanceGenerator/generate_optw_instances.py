"""generate op instances to required format"""
"""op_n_i.txt: x_coordinate y_coordinate score"""
"""op_n_ngSets.txt: e.g., v1 \t  v2 v3 \t v3 v1 represents {v1} for v1, {v2, v3} for v2, {v3, v1} for v3}"""
import random
import os
from op_instance_settings import *
import math


def generate_op_Cordeau_cluster(dir, n_instances, n_centers, n_vertices,fi = 0.05):
    # %% Generate random Cordeau instances in the way defined in Ref. "A Tabu Search Heuristic for Periodic and Multi-Depot Vehicle Routing Problems"
    for i in range(n_instances):
        centers = []
        for t in range(n_centers):
            x_mid = round(random.uniform(center_x_min, center_x_max), 5)
            y_mid = round(random.uniform(center_y_min, center_y_max), 5)
            centers.append((x_mid, y_mid))
            print("center:", centers[t])

        with open(dir+f"c{n_centers}-{n_vertices}-{i+1}.inst", "w") as file:
            # output the depot data as the first line
            file.write(str(n_vertices) + "\n")
            vertices_count = 0
            while vertices_count < n_vertices:
                # Generate x and y coordinates uniformly at random from [0, 1]
                x = round(random.uniform(x_min, x_max), 5)
                y = round(random.uniform(y_min, y_max), 5)
                if vertices_count == 0:
                    score = 0
                else:
                    score = random.randint(score_min, score_max)
                    # calculate the distance to the nearest center, and accept it according to a random probability
                    min_distance = float('inf')
                    for t in range(n_centers):
                        distance = get_distance(x, y, centers[t][0], centers[t][1])
                        if distance < min_distance:
                            min_distance = distance
                    u = random.random()

                    if u >= math.exp(-fi * min_distance):
                        continue
                    # print(f"{min_distance}, {math.exp(-fi * min_distance)}")

                # Write the generated data to the file
                line = " ".join(map(str, [x, y, score]))
                file.write(line + "\n")
                vertices_count += 1


def generate_op_Cordeau_mix(dir, n_instances, n_centers, n_vertices, fi=0.05):
    for i in range(n_instances):
        centers = []
        for t in range(n_centers):
            x_mid = round(random.uniform(center_x_min, center_x_max), 5)
            y_mid = round(random.uniform(center_y_min, center_y_max), 5)
            centers.append((x_mid, y_mid))
            print("center:", centers[t])

        with open(dir + f"rc{n_centers}-{n_vertices}-{i + 1}.inst", "w") as file:
            # output the depot data as the first line
            file.write(str(n_vertices) + "\n")
            vertices_count = 0
            while vertices_count < n_vertices:
                # Generate x and y coordinates uniformly at random from [0, 1]
                x = round(random.uniform(x_min, x_max), 5)
                y = round(random.uniform(y_min, y_max), 5)
                if vertices_count == 0:
                    score = 0
                else:
                    score = random.randint(score_min, score_max)
                    r = random.random() # 一半可能uniform，一半可能cluster
                    if r < 0.5:
                        # calculate the distance to the nearest center, and accept it according to a random probability
                        while True:
                            min_distance = float('inf')
                            for t in range(n_centers):
                                distance = get_distance(x, y, centers[t][0], centers[t][1])
                                if distance < min_distance:
                                    min_distance = distance
                            u = random.random()

                            if u >= math.exp(-fi * min_distance):
                                x = round(random.uniform(x_min, x_max), 5)
                                y = round(random.uniform(y_min, y_max), 5)
                                continue
                            else:
                                break


                    # print(f"{min_distance}, {math.exp(-fi * min_distance)}")

                # Write the generated data to the file
                line = " ".join(map(str, [x, y, score]))
                file.write(line + "\n")
                vertices_count += 1

def generate_op_Cordeau_uniform(dir, n_instances, n_vertices):
    # %% Generate random Cordeau instances in the way defined in Ref. "A Tabu Search Heuristic for Periodic and Multi-Depot Vehicle Routing Problems" with uniform
    if not os.path.exists(dir):
        os.makedirs(dir)
    for i in range(n_instances):
        with open(dir+f"/r-{n_vertices}-{i+1}.inst", "w") as file:
            file.write(str(n_vertices) + "\n")
            vertices_count = 0
            while vertices_count < n_vertices:
                # Generate x and y coordinates uniformly at random from [0, 1]
                x = round(random.uniform(x_min, x_max), 5)
                y = round(random.uniform(y_min, y_max), 5)
                if vertices_count == 0:
                    score = 0
                    # output the depot data as the first line
                else:
                    score = random.randint(score_min, score_max)

                # Write the generated data to the file
                line = " ".join(map(str, [x, y, score]))
                file.write(line + "\n")
                vertices_count += 1

def get_distance(x1, y1, x2, y2):
    return math.floor(math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)*(10**distance_decimal))/(10**distance_decimal)



if __name__ == "__main__":
    # %% parameters of generated instances
    path = './data/mixed/'
    if not os.path.exists(path):
        os.makedirs(path)

    n_instances = 5
    n_vertices = 125
    distance_decimal = 2

    # for cluster distribution
    n_centers = 1
    fi = 0.2

    # generate_op_Cordeau_cluster(path, n_instances, n_centers, n_vertices, fi) # generate Cordeau instances with cluster distribution
    # generate_op_Cordeau_uniform(path, n_instances, n_vertices) # generate Cordeau instances with uniform distribution
    generate_op_Cordeau_mix(path, n_instances, n_centers, n_vertices, fi)
