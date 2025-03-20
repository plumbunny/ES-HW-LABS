import socket
import matplotlib.pyplot as plt
import numpy as np

# === TCP Server Setup ===
HOST = "0.0.0.0"  # Accept connections from any IP
PORT = 8002  # Must match STM32's RemotePORT

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)
print(f"TCP Server started on port {PORT}, waiting for connection...")

conn, addr = server.accept()
print(f"Connected by {addr}")

# === Data Storage for Plotting ===
max_data_points = 100  # Keep only the last 100 points for smooth scrolling
x_data = np.arange(max_data_points)
y_data_x = np.zeros(max_data_points)
y_data_y = np.zeros(max_data_points)
y_data_z = np.zeros(max_data_points)

# === Matplotlib Setup ===
plt.ion()  # Interactive mode on
fig, ax = plt.subplots()

# Create three line plots for X, Y, Z axes
line_x, = ax.plot(x_data, y_data_x, 'r-', label="Accel X")
line_y, = ax.plot(x_data, y_data_y, 'g-', label="Accel Y")
line_z, = ax.plot(x_data, y_data_z, 'b-', label="Accel Z")

ax.set_xlim(0, max_data_points)  # Fixed width for scrolling effect
ax.set_ylim(-1500, 1500)  # Adjust based on expected sensor values
ax.set_xlabel("Time")
ax.set_ylabel("Acceleration (mg)")
ax.set_title("Real-time Accelerometer Data")
ax.legend()


def update_plot():
    """ Updates the Matplotlib plot with new X, Y, Z acceleration data. """
    line_x.set_ydata(y_data_x)
    line_y.set_ydata(y_data_y)
    line_z.set_ydata(y_data_z)

    ax.relim()
    ax.autoscale_view()
    plt.draw()
    plt.pause(0.001)  # Small pause for GUI updates


# === Main Data Processing Loop ===
while True:
    try:
        data = conn.recv(1024)
        if not data:
            break  # Client disconnected
        decoded_data = data.decode().strip()
        print(f"Received: {decoded_data}")

        # Extract Acceleration X, Y, Z values from received data
        if "Accel: X=" in decoded_data:
            try:
                parts = decoded_data.split()
                accel_x = int(parts[1].split('=')[1].strip())
                accel_y = int(parts[2].split('=')[1].strip())
                accel_z = int(parts[3].split('=')[1].strip())

                # Shift old data left, and append new values
                y_data_x[:-1] = y_data_x[1:]
                y_data_y[:-1] = y_data_y[1:]
                y_data_z[:-1] = y_data_z[1:]

                y_data_x[-1] = accel_x
                y_data_y[-1] = accel_y
                y_data_z[-1] = accel_z

                update_plot()  # Refresh the plot

            except ValueError:
                print("Warning: Could not parse acceleration data.")

    except Exception as e:
        print(f"Error: {e}")
        break

conn.close()
server.close()
plt.ioff()  # Disable interactive mode
plt.show()
