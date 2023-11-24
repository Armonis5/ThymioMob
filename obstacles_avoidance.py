
def avoidance(proximity,state = 0):
    obstThrL = 10      # low obstacle threshold to switch state 1->0
    obstThrH = 20      # high obstacle threshold to switch state 0->1
             # state of the robot: 0 = no obstacle, 1 = obstacle
    for i in range(0, 4):
        if state == 0:
            if proximity[i] > obstThrH:
                state = 1
        elif state == 1:
            if proximity[i] < obstThrL:
                state = 0




