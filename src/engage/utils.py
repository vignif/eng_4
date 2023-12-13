import math

class RandomID:
    characters = "abcdefghijklmnopqrstuvwxyz"
    prime_number = 60466181

    def random_id(n):
        string = ''
        hashed = RandomID.hash_number(n+1)
        for x in range(5):
            charnumber = hashed % 26
            hashed = math.floor(hashed / 26)
            string += RandomID.characters[charnumber]
        return string

    def hash_number(n: int, rounds = 20):
        if rounds <= 0:
            return n
        hashed = (n * RandomID.prime_number) % (26 ** 5)
        return RandomID.hash_number(hashed, rounds - 1)