import itertools
def calculate_best_combination(items,max):
    best_combination = []
    highest_score = 0

    for i in range(1, len(items) + 1):
        combinations = itertools.combinations(items.keys(), i)
        for combination in combinations:
            total_price = sum(items[item]["price"] for item in combination)
            if total_price <= max:
                total_score = sum(items[item]["score"] for item in combination)
                if len(combination) ==5:
                    if total_score > highest_score:
                        highest_score = total_score
                        best_combination = list(combination)

    return best_combination, highest_score

# Example usage
drivers = {
    "max": {"price": 30.2, "score": 45},
    "carlos": {"price": 18.8, "score": 36},
    "sergio": {"price": 21.1, "score": 32},
    "charles": {"price": 19.4, "score": 22},
    "george": {"price": 19.1, "score": 20},
    "lando": {"price": 23.1, "score": 16},
    "lewis": {"price": 19.4, "score": 12},
    "zhou": {"price": 7.1, "score": 11},
    "oscar": {"price": 19.1, "score": 10},
    "lance": {"price": 11.2, "score": 8},
    "esteban": {"price": 8.3, "score": 7},
    "fernando": {"price": 15.9, "score": 7},
    "kevin": {"price": 6.7, "score": 7},
    "pirerre": {"price": 7.7, "score": 6},
    "daniel": {"price": 8.9, "score": 5},
    "logan": {"price": 5.4, "score": 3}
    
}
constructor = {
    "RBR": {"price": 28, "score": 89},
    "Ferra": {"price": 19.6, "score": 73},
    "Merce": {"price": 20.2, "score": 42},
    "Mclaren": {"price": 23.3, "score": 36},
    "Aston": {"price": 14.1, "score": 20},
    "Alpine": {"price": 8.3, "score": 12},
    "Kick": {"price": 6.5, "score": 10},
    "haas": {"price": 6.2, "score": 9},
    "RB": {"price": 8.4, "score": 7},
    "Williams": {"price": 6.2, "score": 4}
}

def best():
    max = 0
    best_combo = {"con": [], "driv": []}
    for i in constructor:
        for j in constructor:
            if i == j:
                continue
            score = 0
            price = 0
            score += constructor[i]["score"] + constructor[j]["score"]
            price += constructor[i]["price"] + constructor[j]["price"]
            drive,highest_score = calculate_best_combination(drivers,102.3-price)
            score+=highest_score+drivers[drive[0]]["score"]
            if score > max:
                max = score
                best_combo = {"con": [i,j], "driv": drive}
    return best_combo,max
best_combination,max = best()
print(best_combination,max)