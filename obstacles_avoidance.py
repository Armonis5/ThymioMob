
from threading import Timer

class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

def motors(l_speed,r_speed):
    return {
        "motor.left.target": [l_speed],
        "motor.right.target": [r_speed],
    }

def get_proximty(node,proximity):
    proximity.append(
        {
            node["prox.horizontal"][0],
            node["prox.horizontal"][1],
            node["prox.horizontal"][2],
            node["prox.horizontal"][3],
            node["prox.horizontal"][4],
            node["prox.horizontal"][5],
            node["prox.horizontal"][6],
        }
    )


def avoidance(node,proximity):
    obstacle = False
    for i in range(0, 7):
        if proximity[i] > 2000:
            obstacle = True
            break
    if obstacle:
        node.send_set_variables(motors(0,0))




