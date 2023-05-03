import time


class Timer:
    def __init__(self, t_peroid):
        self.old_time = time.time()
        self.peroid = t_peroid

    def is_fire(self):
        if time.time() - self.old_time >= self.peroid:
            self.old_time = time.time()
            return True
        else: return False

