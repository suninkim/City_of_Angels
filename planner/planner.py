import numpy


class TaskPlanner:
    def __init__(self, config):
        need_initial_ask = config["need_ask"]

        self.next_action_question = config["next_action_question"]
        self.done_action_list = []
        a = 1
        self.vqa_model = None
        self.llm_model = None

    def set_task(self, action_list):
        self.init_prompt = "as"

    def get_next_task(self, image):
        current_env_text = self.vqa_model(image, self.next_action_question)
        next_action = self.llm_model(current_env_text)
