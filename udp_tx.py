import socket
import datetime
import time
import pygame
import ctypes

# Initialize Pygame
pygame.init()

# Initialize the joystick module
pygame.joystick.init()

# Check if joysticks are available
joystick_count = pygame.joystick.get_count()

if joystick_count == 0:
    print("No joystick detected.")
else:
    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Main loop to continuously read joystick input
    running = True
    # Print the current date and time
    # print("Current date and time:", current_datetime)
    # Create a UDP socket
    tx_time = time.time_ns()
    tx_time_hb = time.time_ns()

    max_data_rate = 20000000
    heartbeat_time = 100000000
    # axes_bytes=[]
    # data_to_tx=[]
    joy_data = 0
    sequence_num = 0
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # Get joystick axes
            axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
            # print("Axes:", axes)

            # Get joystick buttons
            buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
            # print("Buttons:", buttons)

            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # axes_bytes = str(axes).encode('utf-8')
            a = int((axes[0] + 1) * 128)
            b = int((axes[1] + 1) * 128)
            c = int((axes[2] + 1) * 128)
            d = int((axes[3] + 1) * 128)
            e = int(buttons[0])
            f = int(buttons[1])
            g = int(buttons[2])
            h = int(buttons[3])

            a1 = a.to_bytes(1, byteorder="big")
            b1 = b.to_bytes(1, byteorder="big")
            c1 = c.to_bytes(1, byteorder="big")
            d1 = d.to_bytes(1, byteorder="big")
            e1 = e.to_bytes(1, byteorder="big")
            f1 = f.to_bytes(1, byteorder="big")
            g1 = g.to_bytes(1, byteorder="big")
            h1 = h.to_bytes(1, byteorder="big")
            hb = sequence_num.to_bytes(1, byteorder="big")
            joy_data = 0
            joy_data = a1 + b1 + c1 + d1 + e1 + f1 + g1 + h1
            tx_data = joy_data + hb

            if time.time_ns() - tx_time > max_data_rate:
                tx_time = time.time_ns()
                tx_time_hb = tx_time
                sequence_num = sequence_num + 1
                if sequence_num > 255:
                    sequence_num = 0
                udp_socket.sendto(tx_data, ("192.168.0.1", 8080))

                # print("tx", a, b, c, d, e, f, g, h, sequence_num)

        if time.time_ns() - tx_time_hb > heartbeat_time:
            tx_time_hb = time.time_ns()

            sequence_num = sequence_num + 1
            if sequence_num > 255:
                sequence_num = 0

            hb = sequence_num.to_bytes(1, byteorder="big")
            tx_data = joy_data + hb
            udp_socket.sendto(tx_data, ("192.168.0.1", 8080))
            # print("tx hb", a, b, c, d, e, f, g, h, sequence_num)
        # Close the socket
udp_socket.close()
