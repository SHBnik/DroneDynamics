import threading
import matplotlib

#   TODO: write this part with matplotlib

class Viz:
    def __init__(self):
        self.vizthread = threading.Thread(target=self.__dosmth)
        self.vizthread.deamon = True
        self.vizthread.start()

    def __dosmth(self):
        pass