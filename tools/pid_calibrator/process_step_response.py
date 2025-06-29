import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


def load_data(filepath):
    """
    Load CSV with columns: time, speed (RPM), and throttle.
    """
    return pd.read_csv(filepath)


def find_step_indices(u, tol=1e-6):
    """
    Identify indices where the input u changes by more than tol.
    Returns a list of step indices.
    """
    du = u[1:] - u[:-1]
    return list(np.where(np.abs(du) > tol)[0] + 1)


def compute_fopdt(t, y, u, step_idx, end_idx=None):
    """
    Compute FOPDT parameters for a single step between step_idx and end_idx.
    If end_idx is None, use end of data.
    """
    if end_idx is None:
        end_idx = len(u)

    # Pre- and post- step means
    u0 = u[:step_idx].mean()
    u1 = u[step_idx:end_idx].mean()
    delta_u = u1 - u0

    y0 = y[:step_idx].mean()
    yf = y[step_idx:end_idx].mean()
    delta_y = yf - y0

    # Avoid division by zero
    if np.isclose(delta_u, 0):
        return None

    # Process gain
    Kp_proc = delta_y / delta_u

    # Dead-time L: first time >5% of response
    thresh_5 = y0 + 0.05 * delta_y
    post = y[step_idx:end_idx]
    t_post = t[step_idx:end_idx]
    idx_L = np.where(post >= thresh_5)[0]
    if idx_L.size == 0:
        L = np.nan
    else:
        L = t_post[idx_L[0]] - t[step_idx]

    # Time constant tau: time to reach 63.2%
    thresh_632 = y0 + 0.632 * delta_y
    idx_tau = np.where(post >= thresh_632)[0]
    if idx_tau.size == 0:
        tau = np.nan
    else:
        tau = t_post[idx_tau[0]] - t[step_idx] - L

    return {
        'step_time': t[step_idx],
        'delta_u': delta_u,
        'delta_y': delta_y,
        'Kp_proc': Kp_proc,
        'dead_time_L': L,
        'tau': tau
    }


def plot_step_response(t, y, u, step_idx, end_idx=None, params=None, ax=None):
    """
    Plot measured response and optional FOPDT fit for a single step.
    """
    if end_idx is None:
        end_idx = len(u)

    if ax is None:
        fig, ax = plt.subplots()

    # Plot measured
    ax.plot(t, y, label='Measured')

    # Overlay FOPDT if params provided and finite
    if params is not None and params['Kp_proc'] is not None:
        t0 = params['step_time']
        u0 = y[:step_idx].mean()
        Kp_proc = params['Kp_proc']
        delta_u = params['delta_u']
        L = params['dead_time_L']
        tau = params['tau']
        y_model = np.where(
            t < t0 + L,
            u0,
            u0 + Kp_proc * delta_u * (1 - np.exp(-(t - t0 - L) / tau))
        )
        ax.plot(t, y_model, '--', label='FOPDT fit')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Speed (RPM)')
    ax.set_title(f'Step at {t[step_idx]:.2f}s, Δu={delta_u:.2f}')
    ax.legend()


def main(filepath):
    df = load_data(filepath)
    t = df['time'].values
    y = df['speed'].values
    u = df['throttle'].values

    # Plot overall response
    plt.figure()
    plt.plot(t, y)
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (RPM)')
    plt.title('Overall Speed vs Time')
    plt.show()

    plt.figure()
    plt.plot(t, u)
    plt.xlabel('Time (s)')
    plt.ylabel('Throttle')
    plt.title('Overall Throttle vs Time')
    plt.show()

    # Find step indices
    steps = find_step_indices(u)
    if not steps:
        print("No throttle steps detected in data.")
        return

    # Compute and plot each step
    results = []
    for i, idx in enumerate(steps):
        next_idx = steps[i+1] if i+1 < len(steps) else len(u)
        params = compute_fopdt(t, y, u, idx, next_idx)
        if params is None:
            print(f"Step at {t[idx]:.2f}s has zero Δu, skipping.")
            continue
        results.append(params)

        plt.figure()
        plot_step_response(t, y, u, idx, next_idx, params)
        plt.show()

    # Summary
    print("Computed FOPDT parameters for detected steps:")
    for p in results:
        print(f"Step @ {p['step_time']:.2f}s: Kp_proc={p['Kp_proc']:.4f}, "
              f"L={p['dead_time_L']:.4f}s, tau={p['tau']:.4f}s")

if __name__ == '__main__':
    import sys
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} path/to/log.csv")
    else:
        main(sys.argv[1])
