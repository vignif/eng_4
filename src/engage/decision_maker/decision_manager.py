from engage.decision_maker.heuristic_decision_maker import HeuristicDecisionMaker
from engage.decision_maker.random_robot_decision_maker import RandomRobotDecisionMaker
from engage.decision_maker.simple_target_decision_maker import SimpleTargetDecisionMaker
from engage.decision_maker.heuristic_ari_controller import HeuristicARIController
from engage.decision_maker.simple_ari_controller import SimpleARIController
from engage.decision_maker.simple_target_ari_controller import SimpleTargetARIController
from engage.msg import HeuristicStateDecision,RobotStateDecision


class DecisionManager:
    decision_makers = {
        "heuristic":HeuristicDecisionMaker,
        "random_robot":RandomRobotDecisionMaker,
        "simple_target":SimpleTargetDecisionMaker,
    }

    decision_state_msgs = {
        "heuristic":HeuristicStateDecision,
        "random_robot":RobotStateDecision,
        "simple_target":HeuristicStateDecision,
    }

    robot_controllers = {
        "heuristic_ari_controller":HeuristicARIController,
        "simple_ari_controller":SimpleARIController,
        "simple_target_ari_controller":SimpleTargetARIController,
    }

    def __init__(self) -> None:
        pass