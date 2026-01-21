# OLD


# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# from matplotlib.patches import Rectangle, Circle



# def animate_cart_pendulum(t, z, L=0.3, cart_w=0.3, cart_h=0.15, pivot_h=0):
#     x = z[0, :]
#     theta = z[2, :]

#     # --- Figure setup ---
#     fig, ax = plt.subplots(figsize=(10, 4))
#     ax.set_aspect('equal', adjustable='box')
#     ax.grid(True)

#     # Set axis limits based on motion range
#     xpad = 1.0
#     ax.set_xlim(np.min(x) - xpad, np.max(x) + xpad)
#     ax.set_ylim(-0.3, 1.0)

#     # Ground line
#     ground, = ax.plot([np.min(x) - xpad, np.max(x) + xpad], [0, 0], lw=2)

#     # Cart
#     cart = Rectangle((x[0] - cart_w/2, 0), cart_w, cart_h, fill=False, lw=2)
#     ax.add_patch(cart)

#     # Pivot point height
#     pivot_y = cart_h + pivot_h

#     # Pendulum rod and bob
#     rod, = ax.plot([], [], lw=3)
#     bob = Circle((0, 0), 0.03, fill=True)
#     ax.add_patch(bob)

#     # Time text
#     time_text = ax.text(0.02, 0.92, "", transform=ax.transAxes)

#     def init():
#         rod.set_data([], [])
#         bob.center = (x[0], pivot_y)
#         time_text.set_text("")
#         return rod, bob, cart, time_text

#     def update(i):
#         # Cart position
#         cart.set_xy((x[i] - cart_w/2, 0))

#         # Pivot position
#         px = x[i]
#         py = pivot_y

#         # Pendulum tip/bob position
#         bx = px + L * np.sin(theta[i])
#         by = py + L * np.cos(theta[i])

#         # Update drawings
#         rod.set_data([px, bx], [py, by])
#         bob.center = (bx, by)
#         time_text.set_text(f"t = {t[i]:.2f} s")

#         return rod, bob, cart, time_text

#     # Frame step to control speed
#     step = max(1, len(t)//600) 
#     ani = FuncAnimation(fig, update, frames=range(0, len(t), step),
#                         init_func=init, blit=True, interval=20)

#     plt.show()
#     return ani








# if __name__ == '__main__':
    
#     # Start time, end time, and time step in seconds
#     t0 = 0
#     tmax = 120
#     dt = 0.1
#     F = 0 # change if simulating with external force

#     # Set initial conditions
#     theta = np.deg2rad(90) # 0 is top
#     theta_dot = 0
#     x = 0
#     x_dot = 0
#     z0 = np.array([[x], [x_dot], [theta], [theta_dot]])
    
#     # Run the solver
#     [t,z] = solve_ivp(t0,tmax,dt,z0, F)
#     ani = animate_cart_pendulum(t, z, L=0.3)



