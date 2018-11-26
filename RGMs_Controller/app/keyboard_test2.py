import keyboard as key


def enter_pressed():
    print('\r' + 'enter is pressed' + ' ' * 40, end='')
    # print('\r' + line + ' ' * 40, end='')


def space_pressed():
    print('\r' + "space is pressed" + ' ' * 40, end='')


def a_pressed():
    print('\r' + "a is pressed" + ' ' * 40, end='')


def d_pressed():
    print("d is pressed")


def key0_pressed():
    global a
    a = 0
    print("A is ", a)


def key1_pressed():
    global a
    print("A is ", a)


def key2_pressed():
    print("2 is pressed")


def key3_pressed():
    print("3 is pressed")


def key4_pressed():
    print("4 is pressed")


def key5_pressed():
    print("5 is pressed")


def key6_pressed():
    print("6 is pressed")


def key7_pressed():
    print("7 is pressed")


def key8_pressed():
    print("8 is pressed")


def key9_pressed():
    print("9 is pressed")


a = 10


def main():
    key.add_hotkey('enter', enter_pressed, args=[])
    key.add_hotkey('space', space_pressed, args=[])
    key.add_hotkey('a', a_pressed, args=[])
    key.add_hotkey('d', d_pressed, args=[])
    call_back_function = [key0_pressed,
                          key1_pressed,
                          key2_pressed,
                          key3_pressed,
                          key4_pressed,
                          key5_pressed,
                          key6_pressed,
                          key7_pressed,
                          key8_pressed,
                          key9_pressed]
    for i in range(10):
        key.add_hotkey(str(i), call_back_function[i], args=[])
    key.wait('esc')
    # print('/r')
    print("end")


main()
