import keyboard as key
import threading


def robot_control():
    global pressed_key_flag
    while True:
        print(pressed_key_flag)
        if pressed_key_flag == 1:
            # key.remove_word_listener()
            pass


def pressed_keys(e):
    global pressed_key_flag
    for code in key._pressed_events:
        pressed_key_flag = code
    pass


def main():
    global pressed_key_flag
    pressed_key_flag = 0

    control_thread = threading.Thread(target=robot_control, args=())
    control_thread.setDaemon(True)
    control_thread.start()

    key.hook(pressed_keys)
    key.wait()


if __name__ == "__main__":
    main()
