import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os

def analyze_throttle_speed_relationship():
    """
    Analyze and plot the relationship between throttle and steady-state speed
    from calibration CSV files.
    """
    
    # Find all throttle calibration files
    csv_files = glob.glob("throttle_*_speed_log.csv")
    csv_files.sort()  # Sort by filename
    
    if not csv_files:
        print("No throttle calibration files found!")
        return
    
    throttle_values = []
    steady_state_speeds = []
    speed_std_devs = []
    
    print("Processing calibration files:")
    print("-" * 50)
    
    for file in csv_files:
        # Extract throttle value from filename
        throttle = int(file.split('_')[1])
        
        # Load data
        df = pd.read_csv(file)
        
        # Remove outliers using IQR method
        Q1 = df['speed'].quantile(0.25)
        Q3 = df['speed'].quantile(0.75)
        IQR = Q3 - Q1
        lower_bound = Q1 - 1.5 * IQR
        upper_bound = Q3 + 1.5 * IQR
        
        # Filter out outliers and zero speeds (motor stopped)
        clean_data = df[(df['speed'] >= lower_bound) & 
                       (df['speed'] <= upper_bound) & 
                       (df['speed'] > 0)]
        
        if len(clean_data) > 0:
            # Calculate steady-state speed (mean of last 50% of data points)
            last_half = clean_data.iloc[len(clean_data)//2:]
            steady_speed = last_half['speed'].mean()
            speed_std = last_half['speed'].std()
            
            throttle_values.append(throttle)
            steady_state_speeds.append(steady_speed)
            speed_std_devs.append(speed_std)
            
            print(f"Throttle {throttle:2d}%: Speed = {steady_speed:6.1f} ± {speed_std:4.1f} RPM")
        else:
            print(f"Throttle {throttle:2d}%: No valid data (motor stopped)")
    
    # Create the plot
    plt.figure(figsize=(12, 8))
    
    # Main scatter plot with error bars
    plt.errorbar(throttle_values, steady_state_speeds, yerr=speed_std_devs, 
                fmt='bo-', capsize=5, capthick=2, linewidth=2, markersize=8,
                label='Measured Data')
    
    # Fit polynomial curves
    if len(throttle_values) > 3:
        # Linear fit
        linear_coeffs = np.polyfit(throttle_values, steady_state_speeds, 1)
        linear_fit = np.poly1d(linear_coeffs)
        
        # Polynomial fit (degree 2)
        poly_coeffs = np.polyfit(throttle_values, steady_state_speeds, 2)
        poly_fit = np.poly1d(poly_coeffs)
        
        # Generate smooth curve for plotting
        throttle_smooth = np.linspace(min(throttle_values), max(throttle_values), 100)
        
        plt.plot(throttle_smooth, linear_fit(throttle_smooth), 'r--', 
                linewidth=2, label=f'Linear fit: y = {linear_coeffs[0]:.2f}x + {linear_coeffs[1]:.2f}')
        plt.plot(throttle_smooth, poly_fit(throttle_smooth), 'g--', 
                linewidth=2, label=f'Polynomial fit (deg 2)')
        
        # Calculate R-squared for linear fit
        ss_res = np.sum((steady_state_speeds - linear_fit(throttle_values)) ** 2)
        ss_tot = np.sum((steady_state_speeds - np.mean(steady_state_speeds)) ** 2)
        r_squared = 1 - (ss_res / ss_tot)
        
        print(f"\nLinear fit R² = {r_squared:.4f}")
        print(f"Linear equation: Speed = {linear_coeffs[0]:.2f} × Throttle + {linear_coeffs[1]:.2f}")
    
    # Formatting
    plt.xlabel('Throttle (%)', fontsize=14)
    plt.ylabel('Steady-State Speed (RPM)', fontsize=14)
    plt.title('Throttle vs Speed Relationship\n(Steady-State Response)', fontsize=16)
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=12)
    
    # Add annotations for key points
    for i, (throttle, speed) in enumerate(zip(throttle_values, steady_state_speeds)):
        if throttle % 5 == 0:  # Annotate every 5% throttle
            plt.annotate(f'{speed:.0f}', 
                        (throttle, speed), 
                        textcoords="offset points", 
                        xytext=(0,10), 
                        ha='center', fontsize=10)
    
    plt.tight_layout()
    plt.savefig('throttle_speed_relationship.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    # Create a second plot showing individual test results
    plt.figure(figsize=(15, 10))
    
    # Plot first 6 throttle tests
    for i, file in enumerate(csv_files[:6]):
        throttle = int(file.split('_')[1])
        df = pd.read_csv(file)
        
        plt.subplot(2, 3, i+1)
        plt.plot(df['time'], df['speed'], 'b-', linewidth=1, alpha=0.7)
        plt.axhline(y=steady_state_speeds[throttle_values.index(throttle)] if throttle in throttle_values else 0, 
                   color='r', linestyle='--', linewidth=2, label=f'Steady-state: {steady_state_speeds[throttle_values.index(throttle)]:.1f} RPM' if throttle in throttle_values else 'No data')
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (RPM)')
        plt.title(f'Throttle {throttle}%')
        plt.grid(True, alpha=0.3)
        plt.legend()
    
    plt.tight_layout()
    plt.savefig('individual_throttle_tests.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    # Print summary statistics
    print(f"\n{'='*60}")
    print("SUMMARY STATISTICS")
    print(f"{'='*60}")
    print(f"Throttle range tested: {min(throttle_values)}% - {max(throttle_values)}%")
    print(f"Speed range achieved: {min(steady_state_speeds):.1f} - {max(steady_state_speeds):.1f} RPM")
    print(f"Minimum working throttle: {min(throttle_values)}%")
    print(f"Speed gain: {(max(steady_state_speeds) - min(steady_state_speeds))/(max(throttle_values) - min(throttle_values)):.2f} RPM/%throttle")

if __name__ == "__main__":
    analyze_throttle_speed_relationship()