from engage.msg import HeuristicDecision as HeuristicDecisionMSG
from engage.msg import MotionActivity,EngagementLevel

class HeuristicText:
    def __init__(self) -> None:
        self.dmsg = HeuristicDecisionMSG
        self.motion = MotionActivity
        self.eng_lvl = EngagementLevel