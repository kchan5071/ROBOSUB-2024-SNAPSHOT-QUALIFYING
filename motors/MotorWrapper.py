import can
import math
import numpy as np
import time

class Can_Wrapper:

    '''
        discord: @kialli
        github: @kchan5071
        
        This class is a wrapper for the CAN bus interface. It is used to send commands to the motors.
        has methods:
            different directions and to stop the robot.
            send the commands to the motors.
            clamp the motor values to a reasonable range.
            convert a value to two's complement.
        
    '''

    def __init__(self):
        self.bus = can.Bus(interface='socketcan',channel = 'can0', receive_own_messages=True)

        self.MAX_MOTOR_VAL = 100
    
        #set ~10 for in air, ~30 in water---------------------------------------------------------------
        self.REASONABLE_MOTOR_MAX = 30
        #-------------------------------------------------------------------------------------------------

        self.motors = [
            #LjoyX   LjoyY   RjoyX   RjoyY    Rtrig   Ltrig   LPad       RDpad
            
           #strafe for/bck   yaw     ptch#     up#    down#   Lroll#     Rroll#
            [ 0,      0,       0,     -1,      -1,      1,      1,        -1], # motor 0 (top front left)
            [ 1,     -1,      -1,      0,       0,      0,      0,         0], # motor 1 (bottom front left)
            [ 0,      0,       0,      1,      -1,      1,      1,        -1], # motor 2 (top back left)
            [-1,     -1,      -1,      0,       0,      0,      0,         0], # motor 3 (bottom back left)
            [ 0,      0,       0,      1,      -1,      1,     -1,         1], # motor 4 (top back right)
            [-1,     -1,       1,      0,       0,      0,      0,         0], # motor 5 (bottom back right)
            [ 0,      0,       0,     -1,      -1,      1,     -1,         1], # motor 6 (top front right)
            [ 1,     -1,       1,      0,       0,      0,      0,         0]  # motor 7 (bottom front right)
        ]
        self.input_list = [0, 0, 0, 0, 0, 0, 0, 0]

    def clamp(self, num):
        ret = max(-1 * self.REASONABLE_MOTOR_MAX, num)
        ret = min(self.REASONABLE_MOTOR_MAX, num)
        return ret


    def twos_complement(self, value):
        if (value < 0):
            value = 255 - abs(value)
        return value
        
    def move_forward(self, value):
        print(f"Move Forward {value}")
        self.input_list[1] = self.clamp(self.input_list[1] + -value)
        
    def move_backward(self, value):
        print("Move Backward")
        self.input_list[1] = self.clamp(self.input_list[1] + value)

    def move_left(self, value):
        
        self.input_list[0] = self.clamp(self.input_list[0] + value)

    def move_right(self, value):
        self.input_list[0] = self.clamp(self.input_list[0] + -value)

    def move_up(self, value):
        self.input_list[4] = self.clamp(self.input_list[4] + value)

    def move_down(self, value):
        self.input_list[5] = self.clamp(self.input_list[5] + value)

    def turn_up(self, value):
        print("Turn Up")
        self.input_list[3] = self.clamp(self.input_list[3] + value)

    def turn_down(self, value):
        print("Turn Down")
        self.input_list[4] = self.clamp(self.input_list[4] + value)

    def turn_left(self, value):
        print("Turn Left")
        self.input_list[2] = self.clamp(self.input_list[2] + value)

    def turn_right(self, value):
        print("Turn Right")
        self.input_list[2] = self.clamp(self.input_list[2] + -value)

    def roll_left(self, value):
        print("Roll Left")
        self.input_list[7] = self.clamp(self.input_list[2] + value)

    def roll_right(self, value):
        print("Roll Right")
        self.input_list[7] = self.clamp(self.input_list[2] + -value)


    def stop(self):
        self.input_list = [0,0,0,0,0,0,0,0]

    def reset(self):
        self.input_list = [0,0,0,0,0,0,0,0]

    def send_command(self):
        thrust_list = []
        for motor in self.motors:
            thrust_list.append(int(self.REASONABLE_MOTOR_MAX * np.dot(motor, self.input_list)))

        motor_value = 0
        command = ""
        for motor_value_from_list in thrust_list:
            motor_value = self.twos_complement(int(self.clamp(motor_value_from_list)))
            command += '{:02X}'.format(motor_value) + " "
        try:
            message = can.Message(arbitration_id = 16, is_extended_id = False, data = bytearray.fromhex(command))
            self.bus.send(message)
            self.reset()
        except:
            print("NON-HEX", command)

#main for testing
def main():
    wrapper = Can_Wrapper()
    # #-----------------------------forward
    # print("move Forward")
    # wrapper.move_forward(.1)
    # wrapper.send_command()
    # time.sleep(10)
    # #-----------------------------stop
    # wrapper.move_backward(.1)
    # wrapper.send_command()
    # time.sleep(1)
    # #-----------------------------backward
    # print("move backward")
    # wrapper.move_backward(.1)
    # wrapper.send_command()
    # time.sleep(1)
    # #-----------------------------stop
    # wrapper.stop()
    # wrapper.send_command()
    # time.sleep(1)



    # #----------------------------left
    # print("Move Left")
    # wrapper.move_left(.1)
    # wrapper.send_command()
    # time.sleep(1)
    # #-----------------------------stop
    # wrapper.move_right(.1)
    # wrapper.send_command()
    # time.sleep(1)
    # #-----------------------------right
    # print("move Right")
    # wrapper.move_right(.1)
    # wrapper.send_command()
    # time.sleep(1)
    # #-----------------------------stop
    # wrapper.stop()
    # wrapper.send_command()
    # time.sleep(1)


    # #-----------------------------up
    # print("Move Up")
    # wrapper.move_up(.1)
    # wrapper.send_command()
    # time.sleep(1)
    # #-----------------------------stop
    # wrapper.move_down(.1)
    # wrapper.send_command()
    # time.sleep(1)
    # #-----------------------------down
    # print("Move Down")
    # wrapper.move_down(.1)
    # wrapper.send_command()
    # time.sleep(1)
    # #-----------------------------stop
    # wrapper.stop()
    # wrapper.send_command()
    # time.sleep(1)


    # #-----------------------------pitch up
    # print("Pitch Up")
    # wrapper.turn_up(.1)
    # wrapper.send_command()
    # time.sleep(10)

    # #-----------------------------stop
    # wrapper.turn_down(.1)
    # wrapper.send_command()
    # time.sleep(1)

    # #-----------------------------pitch down
    # print("PiTcH DoWn")
    # wrapper.turn_down(.1)
    # wrapper.send_command()
    # time.sleep(10)
    # #-----------------------------stop
    # wrapper.stop()
    # wrapper.send_command()
    # time.sleep(1)

    #-----------------------------turn left
    print("TURN left")
    wrapper.turn_left(.1)
    wrapper.send_command()
    time.sleep(10)
    #-----------------------------stop
    wrapper.turn_right(.1)
    wrapper.send_command()
    time.sleep(1)
    #-----------------------------turn right
    print("turn RIGHT")
    wrapper.turn_right(.1)
    wrapper.send_command()
    time.sleep(10)
    #-----------------------------stop
    wrapper.stop()
    wrapper.send_command()
    time.sleep(1)

    #-----------------------------stop
    print("STo0o0o0o0o0P")
    wrapper.stop()
    wrapper.send_command()




if __name__ == "__main__":
    main()
