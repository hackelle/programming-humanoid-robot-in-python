'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello, leftBackToStand, leftBellyToStand, rightBackToStand, rightBellyToStand, wipe_forehead
import pickle

ROBOT_POSE_CLF = "robot_pose.pkl"

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open(ROBOT_POSE_CLF))  # LOAD CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        
        # collect data
        data_strings = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']
        data = []
        for name in data_strings:
            data.append(perception.joint[name])
            
        data.append(perception.imu[0])
        data.append(perception.imu[1])
        
        prediction_index = self.posture_classifier.predict([data])[0] # needs 2d array, give it an array and take index 0
        # get prediction from given list
        posture = ['Right', 'Back', 'Stand', 'Belly', 'HeadBack', 'Sit', 'Knee', 'Left', 'Frog', 'Crouch', 'StandInit'][prediction_index]

        print posture

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    #agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.keyframes = leftBackToStand()
    #agent.keyframes = leftBellyToStand()
    #agent.keyframes = rightBackToStand()
    #agent.keyframes = rightBellyToStand()
    #agent.keyframes = wipe_forehead(1)
    agent.run()
