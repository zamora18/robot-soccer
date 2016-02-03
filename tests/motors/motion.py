import wheelbase as w
import param as p

_x = 0
_y = 0
_theta = 0 # same as heading

def drive(vx,vy,omega):
	# Convert from world to robot velocities
	(OMEGA1, OMEGA2, OMEGA3) = p.get_wheels_angular_velocity(vx, vy, omega, _theta)

	w.Speed(w.M1, OMEGA1)
	w.Speed(w.M2, OMEGA2)
	w.Speed(w.M3, OMEGA3)

def stop():
	w.kill()