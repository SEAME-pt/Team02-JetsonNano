# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt


# def load_data(filepath):
#     """
#     Load CSV with columns: time, speed (RPM), and throttle.
#     """
#     return pd.read_csv(filepath)

# def remove_outliers(df, column='speed', method='iqr', factor=1.5):
#     """
#     Remove outliers from the specified column using IQR method by default.
#     """
#     Q1 = df[column].quantile(0.25)
#     Q3 = df[column].quantile(0.75)
#     IQR = Q3 - Q1
#     mask = (df[column] >= Q1 - factor * IQR) & (df[column] <= Q3 + factor * IQR)
#     print(f"Removed {len(df) - mask.sum()} outliers from {column}")
#     return df[mask].reset_index(drop=True)


# def find_step_indices(u, tol=1e-6):
#     """
#     Identify indices where the input u changes by more than tol.
#     Returns a list of step indices.
#     """
#     du = u[1:] - u[:-1]
#     return list(np.where(np.abs(du) > tol)[0] + 1)


# def compute_fopdt(t, y, u, step_idx, end_idx=None):
#     """
#     Compute FOPDT parameters for a single step between step_idx and end_idx.
#     If end_idx is None, use end of data.
#     """
#     if end_idx is None:
#         end_idx = len(u)

#     # Pre- and post- step means
#     u0 = u[:step_idx].mean()
#     u1 = u[step_idx:end_idx].mean()
#     delta_u = u1 - u0

#     y0 = y[:step_idx].mean()
#     yf = y[step_idx:end_idx].mean()
#     delta_y = yf - y0

#     # Avoid division by zero
#     if np.isclose(delta_u, 0):
#         return None

#     # Process gain
#     Kp_proc = delta_y / delta_u

#     # Dead-time L: first time >5% of response
#     thresh_5 = y0 + 0.05 * delta_y
#     post = y[step_idx:end_idx]
#     t_post = t[step_idx:end_idx]
#     idx_L = np.where(post >= thresh_5)[0]
#     if idx_L.size == 0:
#         L = np.nan
#     else:
#         L = t_post[idx_L[0]] - t[step_idx]

#     # Time constant tau: time to reach 63.2%
#     thresh_632 = y0 + 0.632 * delta_y
#     idx_tau = np.where(post >= thresh_632)[0]
#     if idx_tau.size == 0:
#         tau = np.nan
#     else:
#         tau = t_post[idx_tau[0]] - t[step_idx] - L

#     return {
#         'step_time': t[step_idx],
#         'delta_u': delta_u,
#         'delta_y': delta_y,
#         'Kp_proc': Kp_proc,
#         'dead_time_L': L,
#         'tau': tau
#     }


# def plot_step_response(t, y, u, step_idx, end_idx=None, params=None, ax=None):
#     """
#     Plot measured response and optional FOPDT fit for a single step.
#     """
#     if end_idx is None:
#         end_idx = len(u)

#     if ax is None:
#         fig, ax = plt.subplots()

#     # Plot measured
#     ax.plot(t, y, label='Measured')

#     # Overlay FOPDT if params provided and finite
#     if params is not None and params['Kp_proc'] is not None:
#         t0 = params['step_time']
#         u0 = y[:step_idx].mean()
#         Kp_proc = params['Kp_proc']
#         delta_u = params['delta_u']
#         L = params['dead_time_L']
#         tau = params['tau']
#         y_model = np.where(
#             t < t0 + L,
#             u0,
#             u0 + Kp_proc * delta_u * (1 - np.exp(-(t - t0 - L) / tau))
#         )
#         ax.plot(t, y_model, '--', label='FOPDT fit')

#     ax.set_xlabel('Time (s)')
#     ax.set_ylabel('Speed (RPM)')
#     ax.set_title(f'Step at {t[step_idx]:.2f}s, Δu={delta_u:.2f}')
#     ax.legend()


# def main(filepath, remove_outliers_flag=True, outlier_method='iqr', outlier_factor=1.5):
#     df = load_data(filepath)
    
#     # Remove outliers if requested
#     if remove_outliers_flag:
#         print(f"Original data shape: {df.shape}")
#         df = remove_outliers(df, column='speed', method=outlier_method, factor=outlier_factor)
#         print(f"Cleaned data shape: {df.shape}")
    
#     t = df['time'].values
#     y = df['speed'].values
#     u = df['throttle'].values

#     # Plot overall response
#     plt.figure()
#     plt.plot(t, y)
#     plt.xlabel('Time (s)')
#     plt.ylabel('Speed (RPM)')
#     plt.title('Overall Speed vs Time (After Outlier Removal)' if remove_outliers_flag else 'Overall Speed vs Time')
#     plt.show()

#     plt.figure()
#     plt.plot(t, u)
#     plt.xlabel('Time (s)')
#     plt.ylabel('Throttle')
#     plt.title('Overall Throttle vs Time')
#     plt.show()

#     # Find step indices
#     steps = find_step_indices(u)
#     if not steps:
#         print("No throttle steps detected in data.")
#         return

#     # Compute and plot each step
#     results = []
#     for i, idx in enumerate(steps):
#         next_idx = steps[i+1] if i+1 < len(steps) else len(u)
#         params = compute_fopdt(t, y, u, idx, next_idx)
#         if params is None:
#             print(f"Step at {t[idx]:.2f}s has zero Δu, skipping.")
#             continue
#         results.append(params)

#         plt.figure()
#         plot_step_response(t, y, u, idx, next_idx, params)
#         plt.show()

#     # Summary
#     print("Computed FOPDT parameters for detected steps:")
#     for p in results:
#         print(f"Step @ {p['step_time']:.2f}s: Kp_proc={p['Kp_proc']:.4f}, "
#               f"L={p['dead_time_L']:.4f}s, tau={p['tau']:.4f}s")

# if __name__ == '__main__':
#     import sys
#     if len(sys.argv) < 2:
#         print(f"Usage: {sys.argv[0]} path/to/log.csv [--no-outlier-removal] [--method zscore] [--factor 2.0]")
#     else:
#         filepath = sys.argv[1]
#         remove_outliers_flag = '--no-outlier-removal' not in sys.argv
        
#         # Parse method
#         method = 'iqr'
#         if '--method' in sys.argv:
#             method_idx = sys.argv.index('--method')
#             if method_idx + 1 < len(sys.argv):
#                 method = sys.argv[method_idx + 1]
        
#         # Parse factor
#         factor = 1.5 if method == 'iqr' else 3.0
#         if '--factor' in sys.argv:
#             factor_idx = sys.argv.index('--factor')
#             if factor_idx + 1 < len(sys.argv):
#                 factor = float(sys.argv[factor_idx + 1])
        
#         main(filepath, remove_outliers_flag, method, factor)

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Feed-forward coefficients
A0 = 28  # throttle units at zero speed
A1 = 0.21   # throttle units per RPM


def load_data(filepath):
    """
    Load CSV with columns: time, speed (RPM), and throttle.
    """
    df = pd.read_csv(filepath)
    return df.dropna(subset=['time', 'speed', 'throttle'])


def remove_outliers(df, column='speed', method='iqr', factor=1.5):
    """
    Remove outliers from the specified column using IQR method by default.
    """
    Q1 = df[column].quantile(0.25)
    Q3 = df[column].quantile(0.75)
    IQR = Q3 - Q1
    mask = (df[column] >= Q1 - factor * IQR) & (df[column] <= Q3 + factor * IQR)
    print(f"Removed {len(df) - mask.sum()} outliers from {column}")
    return df[mask].reset_index(drop=True)


def find_step_indices(u, tol=1e-6, min_interval=5):
    """
    Identify indices where the input u changes by more than tol,
    enforcing a minimum interval between steps in samples.
    """
    du = np.diff(u)
    candidates = np.where(np.abs(du) > tol)[0] + 1
    steps = []
    last = -min_interval
    for idx in candidates:
        if idx - last >= min_interval:
            steps.append(idx)
            last = idx
    return steps


def compute_fopdt(t, y, u, step_idx, end_idx=None):
    """
    Compute FOPDT parameters for a single step, removing feed-forward.
    Returns a dict with keys: step_time, delta_u_eff, delta_y, Kp_proc, dead_time_L, tau.
    """
    if end_idx is None:
        end_idx = len(u)

    # Raw pre- and post- step means
    u0 = u[:step_idx].mean()
    u1 = u[step_idx:end_idx].mean()
    y0 = y[:step_idx].mean()

    # Steady-state average after step (use second half of interval)
    ss_start = step_idx + int((end_idx - step_idx) * 0.5)
    y_ss = y[ss_start:end_idx].mean()

    # Compute effective throttle by subtracting feed-forward
    u_ff_pre = A0 + A1 * y0
    u_ff_post = A0 + A1 * y_ss
    u0_eff = u0 - u_ff_pre
    u1_eff = u1 - u_ff_post
    delta_u_eff = u1_eff - u0_eff

    # Compute delta_y from steady-state average
    delta_y = y_ss - y0

    # Avoid division by zero
    if np.isclose(delta_u_eff, 0):
        return None

    # Process gain
    Kp_proc = delta_y / delta_u_eff

    # Dead-time L: first time >5% of response (use raw y)
    thresh5 = y0 + 0.05 * delta_y
    post = y[step_idx:end_idx]
    t_post = t[step_idx:end_idx]
    idx_L = np.where(post >= thresh5)[0]
    L = t_post[idx_L[0]] - t[step_idx] if idx_L.size > 0 else np.nan

    # Time constant tau: time to reach 63.2%
    thresh63 = y0 + 0.632 * delta_y
    idx_tau = np.where(post >= thresh63)[0]
    tau = (t_post[idx_tau[0]] - t[step_idx] - L) if idx_tau.size > 0 else np.nan

    return {
        'step_time': t[step_idx],
        'delta_u_eff': delta_u_eff,
        'delta_y': delta_y,
        'Kp_proc': Kp_proc,
        'dead_time_L': L,
        'tau': tau
    }


def plot_step_response(t, y, params, step_idx):
    """
    Plot measured response and FOPDT fit using averaged parameters.
    """
    plt.figure()
    plt.plot(t, y, label='Measured RPM')

    if params:
        t0 = params['step_time']
        L = params['dead_time_L']
        tau = params['tau']
        Kp = params['Kp_proc']
        delta_u_eff = params['delta_u_eff']  # Use directly instead of back-calculating
        y0 = y[:int(step_idx)].mean()
        y_model = np.where(
            t < t0 + L,
            y0,
            y0 + Kp * delta_u_eff * (1 - np.exp(-(t - t0 - L) / tau))
        )
        plt.plot(t, y_model, '--', label='FOPDT fit')

    plt.xlabel('Time (s)')
    plt.ylabel('Speed (RPM)')
    plt.legend()
    if params:
        plt.title(f"Step @ {params['step_time']:.2f}s, Δu_eff={params['delta_u_eff']:.2f}")
    plt.show()


def main(filepath, clean=False):
    df = load_data(filepath)
    if clean:
        df = remove_outliers(df, 'speed')

    t = df['time'].values
    y = df['speed'].values
    u = df['throttle'].values

    steps = find_step_indices(u)
    if not steps:
        print("No throttle steps detected.")
        return

    # Compute and collect parameters
    params_list = []
    for i, idx in enumerate(steps):
        end_idx = steps[i+1] if i+1 < len(steps) else len(u)
        params = compute_fopdt(t, y, u, idx, end_idx)
        if params:
            params_list.append(params)

    if not params_list:
        print("No valid steps for FOPDT calculation.")
        return

    # Average parameters
    Kp_proc = np.nanmean([p['Kp_proc'] for p in params_list])
    L_avg = np.nanmean([p['dead_time_L'] for p in params_list])
    tau_avg = np.nanmean([p['tau'] for p in params_list])
    delta_u_avg = np.nanmean([p['delta_u_eff'] for p in params_list])
    delta_y_avg = np.nanmean([p['delta_y'] for p in params_list])

    print(f"Averaged FOPDT -> Kp_proc: {Kp_proc:.4f}, L: {L_avg:.4f}s, tau: {tau_avg:.4f}s")

    # Plot a single representative step (first) with complete params dict
    avg_params = {
        'step_time': t[steps[0]], 
        'delta_u_eff': delta_u_avg,
        'delta_y': delta_y_avg,
        'Kp_proc': Kp_proc, 
        'dead_time_L': L_avg, 
        'tau': tau_avg
    }
    plot_step_response(t, y, avg_params, steps[0])

if __name__ == '__main__':
    import sys
    filepath = sys.argv[1] if len(sys.argv) > 1 else 'curve_speed_pid_log.csv'
    main(filepath)
