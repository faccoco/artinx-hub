import subprocess  # 使用 subprocess 执行 shell 脚本
import RPi.GPIO as GPIO
from time import sleep

def run_script():
    # 替换 'your_script.sh' 为你的脚本路径
    subprocess.Popen(["/etc/rc.local/poweroff.sh"], shell=True)
    print("run script!\n")


if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(7, GPIO.IN)
    last_button_state = 0
    current_button_state = 0
    first_in = True
    while(1):
        if first_in:
            for i in range(10):
                current_button_state = GPIO.input(7)
                last_button_state = current_button_state
                sleep(0.1)#forbid start
            first_in = False
        
        current_button_state = GPIO.input(7)
        if current_button_state != last_button_state:
            run_script()
            exit()
        last_button_state = current_button_state
        print(GPIO.input(7))
        sleep(0.3)
