import random
import numpy as np
import math
from itertools import permutations
from statistics import *

# ----------------------------------------------------------------------------------------------------------------------
# A class for all objects of type city
class City:
    def __init__(self, city, x, y):
        self.city = city
        self.x = x
        self.y = y

    def __sub__(self, other):
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

# ----------------------------------------------------------------------------------------------------------------------
# In order to generate a number of cities
def generate_cities(nb_cities: int):
    # Returns a list of tuples that are the x,y coordinates of a city
    cities = []
    for i in range(0, nb_cities):
        x = np.random.uniform(0.0, 1.0)
        y = np.random.uniform(0.0, 1.0)
        city = str(i)
        cities.append(City(city, x, y))

    return cities

# ----------------------------------------------------------------------------------------------------------------------
# All the helpful functions needed to complete these problems

def path_distance(list_cities):
    # Takes in a given list of cities and returns the total path distance
    total_distance = 0
    for i in range(0, len(list_cities) - 1):
        total_distance += list_cities[i] - list_cities[i + 1]
    total_distance += list_cities[len(list_cities) - 1] - list_cities[0]
    return total_distance


def brute_force(list_cities):
    # Takes in the total lies of cities and finds the optimal paths with the least amount of total distance
    every_path = list(permutations(list_cities))
    every_distance = []
    for path in every_path:
        every_distance.append(path_distance(path))
    min_dis = min(every_distance)
    optimal_path_list = every_path[every_distance.index(min_dis)]

    optimal_path = []
    for opt_path in optimal_path_list:
        optimal_path.append(opt_path.city)
    return optimal_path, min_dis


def random_tour(list_cities):
    # Takes a list of cities and returns a random path with its distance
    copy_of_list_cities = list_cities.copy()
    random.shuffle(copy_of_list_cities)
    return copy_of_list_cities, path_distance(copy_of_list_cities)


def two_change_nb(path, i, j):
    # Takes a path, cuts it at positions i and k, and inverts the order of corresponding vertices
    new_path = path.copy()
    tmp = path[i:j + 1]
    list.reverse(tmp)
    new_path[i:j + 1] = tmp
    return new_path


def hill_climb(path):
    stuck = True
    counter = 0  # number of random points we will take to see if we are stuck
    bestest_cost = math.inf
    while (stuck):
        cur_path, best_cost = random_tour(path)
        best_path = cur_path.copy()
        n = len(cur_path)
        it_gets_better = True
        while it_gets_better:
            it_gets_better = False
            for i in range(0, n):
                for j in range(i + 1, n + 1):
                    new_path = two_change_nb(best_path, i, j)
                    new_cost = path_distance(new_path)
                    if new_cost < best_cost:
                        best_cost = new_cost
                        best_path = new_path
                        it_gets_better = True
                    if best_cost < bestest_cost:
                        bestest_cost = best_cost
        if math.isclose(bestest_cost, best_cost, abs_tol=.00000001) and counter >= 5:
            stuck = False
        counter += 1

    return best_path, best_cost


# ----------------------------------------------------------------------------------------------------------------------
# All functions that will be used in the main to test and to visualize information
def print_brute_force(nb_cities):
    # prints all the statistics for the brute force function with the wanted statistics
    print("------------------BRUTE FORCE-----------------------")
    paths = []
    costs = []
    for i in range(0, 100):
        cities = generate_cities(nb_cities)
        path, cost = brute_force(cities)
        costs.append(cost)
        paths.append(path)

    print("mean: ", mean(costs))
    print("min-cost: ", min(costs))
    print("max-cost: ", max(costs))
    print("standard deviation: ", stdev(costs))


def print_random_tour(nb_cities, instances):
    # prints all the statistics for the random tour function with the wanted statistics
    print("--------------------RANDOM TOUR------------------------ ")
    optimal_cost = []
    rand_cost = []
    rand_is_opt = 0
    for i in range(0, instances):
        cities = generate_cities(nb_cities)
        (path, cost) = random_tour(cities)
        rand_cost.append(cost)
        (path, cost) = brute_force(cities)
        optimal_cost.append(cost)
        if rand_cost[i] == optimal_cost[i]:
            rand_is_opt += 1

    print("mean: ", mean(rand_cost))
    print("min-cost: ", min(rand_cost))
    print("max-cost: ", max(rand_cost))
    print("standard deviation: ", stdev(rand_cost))
    print("Optimal solutions: ", rand_is_opt)


def print_hill_climbing(num, instances):
    print("---------------------------HILL CLIMBING: -------------------------------")
    optimal_cost = []
    hc_cost = []
    hc_is_opt = 0
    for i in range(0, instances):
        cities = generate_cities(num)
        path, cost = hill_climb(cities)
        hc_cost.append(cost)
        path, cost = brute_force(cities)
        optimal_cost.append(cost)
        # print("Cost of hill climbing " + str(hc_cost[i]) + " Optimal cost from brute force " + str(optimal_cost[i]))
        if math.isclose(optimal_cost[i], hc_cost[i], abs_tol=0.0000001):
            hc_is_opt += 1

    print("mean: ", mean(hc_cost))
    print("min-cost: ", min(hc_cost))
    print("max-cost: ", max(hc_cost))
    print("standard deviation: ", stdev(hc_cost))
    print("Optimal solutions: ", hc_is_opt)


def print_big_random_tour(nb_cities, instances):
    # prints all the statistics for the random tour function with the wanted statistics
    print("--------------------BIG RANDOM TOUR------------------------ ")
    rand_cost = []
    for i in range(0, instances):
        cities = generate_cities(nb_cities)
        (path, cost) = random_tour(cities)
        rand_cost.append(cost)


    print("mean: ", mean(rand_cost))
    print("min-cost: ", min(rand_cost))
    print("max-cost: ", max(rand_cost))
    print("standard deviation: ", stdev(rand_cost))




def print_big_hill_climbing(num, instances):
    print("---------------------------BIG HILL CLIMBING: -------------------------------")
    hc_cost = []
    for i in range(0, instances):
        cities = generate_cities(num)
        path, cost = hill_climb(cities)
        hc_cost.append(cost)

    print("mean: ", mean(hc_cost))
    print("min-cost: ", min(hc_cost))
    print("max-cost: ", max(hc_cost))
    print("standard deviation: ", stdev(hc_cost))



def main():
    print("\n Question a)")
    print_brute_force(7)

    print("\n Question b)")
    print_random_tour(7, 100)

    print("\n Question c)")
    print_hill_climbing(7, 100)


    print("\n Question d)")
    
    print_big_random_tour(100, 100)

    print_big_hill_climbing(100, 100)
    
    #These two tests take forever, keep them for later



if __name__ == "__main__":
    main()
