import math
import numpy as np

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
    
class VectorHelper:
    def normalise(v):
        norm = np.linalg.norm(v)
        if norm == 0: 
            return v
        return v / norm
    
    def get_normal(points):
        t1 = points[1]-points[0]
        t2 = points[2]-points[0]
        n = np.cross(t1,t2)
        return VectorHelper.normalise(n)
    
    def obj_to_vec(obj):
        if obj is None:
            return None
        return np.array([obj.x,obj.y,obj.z])
    
    def distance(a,b):
        if a is None or b is None:
            return None
        return np.linalg.norm(a-b)
    
    def angle_between(vec1,vec2):
        return np.arccos((np.dot(vec1,vec2))/(np.linalg.norm(vec1)*np.linalg.norm(vec2)))
    
    def facing_point(vector,a,b,angle):
        # True if the cone projected from a with vector at its centre (radius angle in degrees) contains point b
        theta = angle/180*np.pi
        if vector is None or a is None or b is None:
            return False
        angle_ab = VectorHelper.angle_between(b-a,vector)
        if np.arccos(angle_ab)<=theta:
            return True
        return False