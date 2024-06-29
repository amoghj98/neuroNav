import numpy as np

def compute_velocity(positions, delta_t):
    """ Compute velocities from position data """
    velocities = np.diff(positions, axis=0) / delta_t
    return velocities

def compute_energy(velocities, params):
    """ Calculate energy based on velocities and provided model constants """
    vx, vy, vz = velocities[:, 0], velocities[:, 1], velocities[:, 2]
    # Compute energy for each component separately and then sum them
    energy_x = np.sum(params['c1'] + params['c2'] * vx + params['c3'] * vx**2 +
                      params['c4'] * vx**3 + params['c5'] * vx**4 + params['c6'] * vx**5)
    energy_y = np.sum(params['c1'] + params['c2'] * vy + params['c3'] * vy**2 +
                      params['c4'] * vy**3 + params['c5'] * vy**4 + params['c6'] * vy**5)
    energy_z = np.sum(params['c1'] + params['c2'] * vz + params['c3'] * vz**2 +
                      params['c4'] * vz**3 + params['c5'] * vz**4 + params['c6'] * vz**5)
    
    total_energy = energy_x + energy_y + energy_z  # Sum energies from all components
    return total_energy


def identify_dynamic_segments(velocities, threshold):
    """ Identify segments where velocity magnitude exceeds a threshold """
    speeds = np.sqrt(np.sum(velocities**2, axis=1))
    dynamic_indices = np.where(speeds > threshold)[0]
    return dynamic_indices

def load_positions_compute_dynamic_energy(file_path_x, file_path_y, file_path_z, params, velocity_threshold):
    """ Load position data, compute velocities and dynamic energy """
    x = np.load(file_path_x)
    y = np.load(file_path_y)
    z = np.load(file_path_z)
    positions = np.vstack((x, y, z)).T  # Combine into a single array
    
    delta_t = 1 / 60.0  # Assuming uniform sampling at 60Hz
    velocities = compute_velocity(positions, delta_t)
    
    dynamic_indices = identify_dynamic_segments(velocities, velocity_threshold)
    if len(dynamic_indices) > 0:
        dynamic_velocities = velocities[dynamic_indices]
        energy = compute_energy(dynamic_velocities, params)
        return energy
    return 0

# Constants and Parameters
c1 = 0.255 
c2 = 0.014
c3 = 0.0011
c4 = 0.000074
c5 = 0.0000029
c6 = 0.000000058  # c5 as c6
params = {'c1': c1, 'c2': c2, 'c3': c3, 'c4': c4, 'c5': c5, 'c6': c6}
velocity_threshold = 0.05  # Define based on observed velocity data

# Energy computation focusing on dynamic segments
ev_planner_energy = load_positions_compute_dynamic_energy(
    '/home/sourav/icra40/EV-Planner/xPos.npy',
    '/home/sourav/icra40/EV-Planner/yPos.npy',
    '/home/sourav/icra40/EV-Planner/zPos.npy', params, velocity_threshold)
pid_energy = load_positions_compute_dynamic_energy(
    '/home/sourav/icra40/PID/xPos.npy',
    '/home/sourav/icra40/PID/yPos.npy',
    '/home/sourav/icra40/PID/zPos.npy', params, velocity_threshold)

print("Dynamic Energy consumed by EV-Planner: ", ev_planner_energy)
print("Dynamic Energy consumed by PID: ", pid_energy)
