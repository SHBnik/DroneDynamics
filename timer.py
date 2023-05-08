import time


class Timer:
    def __init__(self, t_peroid):
        self.first_time = time.time()
        self.old_time = time.time()
        self.peroid = t_peroid

    def is_fire(self):
        if time.time() - self.old_time >= self.peroid:
            self.old_time = time.time()
            return True
        else: return False

    def current_time(self):
        return time.time() - self.first_time

    def small_delay(self):
        time.sleep(1e-6)


class Counter:
    def __init__(self):
        self.old_time = time.time()
    
    def stop(self):
        return time.time() - self.old_time
