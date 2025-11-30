import time
import sys
from pynput import keyboard
from orca_core import OrcaHand

# ================= 核心参数配置 =================
MODEL_PATH = "orca_core/models/orcahand_v1_left"
CONTROL_FREQUENCY = 10
STEP_LIMIT = 1 
TARGET_AMPLITUDE = 1
# ==============================================

class OrcaController:
    def __init__(self):
        self.mapping = {
            'g': 1, 'h': -1,  
            'd': 2, 'f': -2,  
            'e': 3, 'r': -3,  
            'c': 4, 'v': -4,  
            '[': 5, ']': -5,  
            '.': 6, '/': -6,  
            'p': 7, 'o': -7,  
            ';': 8, 'l': -8,  
            't': 9, 'y': -9,
            'a': 10, 's': -10,
            'q': 11, 'w': -11,
            'z': 12, 'x': -12,
            'b': 13, 'n': -13,
            'm': 14, ',': -14,
            'j': 15, 'k': -15, 
            'u': 16, 'i': -16,
        }
        self.sign = 1
        self.running = True
        self.pressed_keys = set()
        self.is_torque_on = False 
        
        # 这里的 value 代表“每次循环增加的量”
        # 0 代表不动，1 代表增加1，-1 代表减少1
        self.target_increments = {abs(v): 0 for v in self.mapping.values()}
    

        print(f"Connecting to hand at {MODEL_PATH}...")
        self.hand = OrcaHand(MODEL_PATH)
        status = self.hand.connect()
        self.hand.set_control_mode("step_motor")
        
        if not status[0]:
            print(f"Error: Failed to connect. Status: {status}")
            sys.exit(1)
            
        self.hand.disable_torque()
        print("Connection successful!")
        print("-" * 40)
        print(f"Relative Mode. Freq: {CONTROL_FREQUENCY}Hz")
        print("Controls: Hold keys to move, Release to stop.")
        print("-" * 40)

    def on_press(self, key):
        try:
            if hasattr(key, 'char') and key.char:
                k = key.char
                if k == '0':
                    self.running = False
                    return False
                
                if k in self.mapping:
                    self.pressed_keys.add(k)
                    
                    val = self.mapping[k]
                    motor_id = abs(val)
                    direction = 1 if val > 0 else -1
                    
                    # 按下时，设置增量为 1 或 -1
                    self.target_increments[motor_id] = int(TARGET_AMPLITUDE * direction)
                    
                    # 打印方向提示
                    if direction > 0:
                        print(f"\rMotor {motor_id}: + (Moving)   ", end="", flush=True)
                    else:
                        print(f"\rMotor {motor_id}: - (Moving)   ", end="", flush=True)

        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if hasattr(key, 'char') and key.char in self.mapping:
                k = key.char
                self.pressed_keys.discard(k)
                
                # ==========================================
                # 【关键修复】
                # 1. 获取映射值 (例如 'w' -> -1)
                val = self.mapping[k]
                # 2. 取绝对值获取电机ID (abs(-1) -> 1)
                motor_id = abs(val)
                # 3. 将该电机的增量重置为 0
                self.target_increments[motor_id] = 0
                # ==========================================
                
        except AttributeError:
            pass

    def run(self):
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        try:
            while self.running:
                has_active_input = len(self.pressed_keys) > 0

                if has_active_input:
                    # 1. 如果有键按下，开启力矩
                    if not self.is_torque_on:
                        self.hand.enable_torque()
                        self.is_torque_on = True

                    # 2. 发送相对位置指令
                    # 此时 target_increments 里，没按的键是 0，按下的键是 1 或 -1
                    # rel_to_current=True 意味着：当前位置 + 0 (不动) 或 当前位置 + 1 (移动)
                    self.hand._set_motor_pos(self.target_increments, rel_to_current=True)
                
                else:
                    # 3. 如果没有键按下，关闭力矩（或者停止发送指令）
                    if self.is_torque_on:
                        print("\rStopped. (Torque OFF)        ", end="", flush=True)
                        self.hand.disable_torque()
                        self.is_torque_on = False
                        
                        # 再次确保所有增量归零（双重保险）
                        for mid in self.target_increments:
                            self.target_increments[mid] = 0

                time.sleep(1.0 / CONTROL_FREQUENCY)

        except KeyboardInterrupt:
            pass
        finally:
            self.hand.disable_torque()
            listener.stop()

if __name__ == "__main__":
    controller = OrcaController()
    controller.run()