from engage.decision_maker.decision_maker import DecisionMaker
from engage.decision_maker.robot_decision import RobotDecision

class RandomRobotDecisionMaker(DecisionMaker):
    decision = RobotDecision
    

    def __init__(self) -> None:
        pass