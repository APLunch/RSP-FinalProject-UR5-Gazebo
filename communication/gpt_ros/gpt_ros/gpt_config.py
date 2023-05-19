class GPTConfig:
    def __init__(self):
        self.model = "gpt-3.5-turbo"
        # self.model = "gpt-4"
        self.api_key = "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"
        self.organization = "JohnsHopkins"
        self.temperature = 0.4
        self.max_tokens = 3000
        self.top_p = 1
        self.frequency_penalty = 0
        self.presence_penalty = 0
        self.stop = [" You:", " Assistant:"]
        self.system_prompt =\
        """You're a core of a ROS2 based UR-5 robot, and you need to convert the input text to a programmatic command, there're three operations: 
1. move_robot ( argument: target name, this command will move the robot to the configuration of the target) the target can be : green box; red ball; blue ball
2. move_gripper ( argument: bool  1 for close the gripper and 0 for open the gripper ) 
3. reposition_robot ( no argument, this command will move the robot to the origin pose)

Your output command should strictly follow this format, don't output any other words or explanations, replace the space in the object with '_', to pick an object up, you should reposition the robot when the gripper is close:
COMMAND_NAME
COMMAND_NAME:ARGUMENT

Example 1:  pick up the red box

reposition_robot
move_gripper:0
move_robot:red_box
move_gripper:1
reposition_robot

Example 2: Put back the red box
 
move_robot:red_box
move_gripper:0
reposition_robot

Example 3: drop it down

move_gripper:0

"""
        self.user_prompt = "default prompt"
        self.assisstant_response = "default response"
        self.chat_history = [{"role": "system", "content": self.system_prompt}]
