def forward_thread(drone, amount):
    drone.tello.move_forward(amount)

def backward_thread(drone, amount):
    drone.tello.move_back(amount)

def cw_thread(drone, amount):
    drone.tello.rotate_clockwise(amount)

def ccw_thread(drone, amount):
    drone.tello.rotate_counter_clockwise(amount)

def left_thread(drone, amount):
    drone.tello.move_left(amount)

def right_thread(drone, amount):
    drone.tello.move_right(amount)

def up_thread(drone, amount):
    drone.tello.move_up(amount)

def down_thread(drone, amount):
    drone.tello.move_down(amount)