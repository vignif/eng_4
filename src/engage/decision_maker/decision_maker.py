from engage.msg import HeuristicDecision as HeuristicDecisionMSG

class Decision:
    def __init__(self) -> None:
        pass

    def message(self,time):
        # return the associated message object
        raise NotImplementedError
    
    @staticmethod
    def create_publisher(self,topic,queue_size):
        raise NotImplementedError

class DecisionState:
    def __init__(self) -> None:
        pass

    def message(self,decision:Decision):
        # return the associated message object
        raise NotImplementedError
    
    def check_group(self,id):
        # True if id's group > 1 person
        same_group = [k for k,v in self.groups.items() if v == self.groups[id]]
        return len(same_group)>1
    
    @staticmethod
    def create_publisher(msg,topic,queue_size=1):
        raise NotImplementedError

    @staticmethod
    def float_bucket(value):
        if value < 0.25:
            return 0
        elif value < 0.5:
            return 1
        elif value < 0.75:
            return 2
        else:
            return 3

    @staticmethod
    def distance_bucket(distance):
        if distance <= 3:
            discretised_distance = round(distance * 2) / 2
        else:
            discretised_distance = round(distance)

        if discretised_distance == 0.0:
            discretised_distance = 0.1
        elif discretised_distance > 8:
            discretised_distance = 9

        return discretised_distance

class DecisionMaker:
    # Need to override this with the appropriate message type
    decision = HeuristicDecisionMSG

    def __init__(self,wait_time=5) -> None:
        self.wait_time = wait_time
        self.last_decision_time = None

    def decide(self,decision_state:DecisionState):
        raise NotImplementedError
    
    def is_waiting(self,time):
        if self.last_decision_time is None or time - self.last_decision_time > self.wait_time:
            return False
        else:
            return True
        
    def update_last_decision_time(self,decision:Decision,time):
        raise NotImplementedError
    
    
    
class RobotController:
    def __init__(self) -> None:
        pass

    def execute_command(decision:Decision,decision_state:DecisionState):
        raise NotImplementedError