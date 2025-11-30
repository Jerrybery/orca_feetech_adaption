import time
import sys
from pynput import keyboard
from orca_core import OrcaHand

# ================= 配置区域 =================
MODEL_PATH = "orca_core/models/orcahand_v1_left"
CONTROL_FREQUENCY = 20  # Hz
# ===========================================

class OrcaController:
    def __init__(self):
        self.mapping = {
            '1': 1, '2': 2, '3': 3, '4': 4, '5': 5,
            '6': 6, '7': 7, '8': 8, '9': 9, '0': 10,
            '-': 11, '=': 12, '\\': 13, '[': 14, ']': 15, 'p': 16
        }
        
        self.sign = 1
        self.running = True
        
        # 核心修改：使用集合记录当前按下的“电机控制键”
        self.pressed_keys = set()
        
        # 记录当前的力矩状态，避免重复发送 enable/disable 指令
        self.is_torque_on = False 
        
        self.target_positions = {}
        for motor_id in self.mapping.values():
            self.target_positions[motor_id] = 0

        print(f"Connecting to hand at {MODEL_PATH}...")
        self.hand = OrcaHand(MODEL_PATH)
        status = self.hand.connect()
        
        if not status[0]:
            print(f"Error: Failed to connect. Status: {status}")
            sys.exit(1)
            
        print("Connection successful!")
        
        # 初始状态：关闭力矩（软手状态）
        self.hand.disable_torque()
        print("Torque is currently DISABLED (Passive mode).")
        print("-" * 40)
        print("Controls:")
        print("  [Hold Keys] : Enable torque and move motor")
        print("  [Release]   : Disable torque (Hand goes limp)")
        print("  [c] / [C]   : Change direction (+/-)")
        print("  [q]         : Quit")
        print("-" * 40)

    def on_press(self, key):
        try:
            if hasattr(key, 'char') and key.char:
                k = key.char
                
                if k == 'q':
                    self.running = False
                    return False

                if k == 'c':
                    self.sign = 1
                    print(f"\rDirection: Positive (+) ", end="", flush=True)
                elif k == 'C':
                    self.sign = -1
                    print(f"\rDirection: Negative (-) ", end="", flush=True)
                
                elif k in self.mapping:
                    # 将按键加入集合
                    self.pressed_keys.add(k)
                    
                    # 更新目标位置
                    motor_id = self.mapping[k]
                    self.target_positions[motor_id] = 5 * self.sign

        except AttributeError:
            pass

    def on_release(self, key):
        try:
            self.pressed_keys = set()
                                
        except AttributeError:
            pass

    def run(self):
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        try:
            while self.running:
                # === 状态机逻辑 ===
                
                # 检查是否有任何电机控制键被按下
                has_active_input = len(self.pressed_keys) > 0

                if has_active_input:
                    # 1. 如果之前没开力矩，现在开启
                    if not self.is_torque_on:
                        print("\rInput detected -> Enabling Torque", end="", flush=True)
                        self.hand.enable_torque()
                        self.is_torque_on = True
                        # 给一点点时间让硬件反应（可选）
                        time.sleep(0.02) 

                    # 2. 发送运动指令
                    self.hand._set_motor_pos(self.target_positions, rel_to_current=True)
                
                else:
                    # 没有按键按下
                    # 如果之前开着力矩，现在关闭
                    if self.is_torque_on:
                        print("\rNo input -> Disabling Torque     ", end="", flush=True)
                        self.hand.disable_torque()
                        self.is_torque_on = False
                    
                    # 此时不发送 _set_motor_pos，因为力矩已关，发了也没用

                time.sleep(1.0 / CONTROL_FREQUENCY)
                
        except KeyboardInterrupt:
            pass
        finally:
            print("\nStopping...")
            self.hand.disable_torque() # 确保退出时松力
            listener.stop()
            print("Exited.")

if __name__ == "__main__":
    controller = OrcaController()
    controller.run()