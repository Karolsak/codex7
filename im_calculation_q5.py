"""
Induction Motor Analysis - Solution to Question 5
Standalone calculation script (no GUI required)
Pure Python implementation - no external dependencies
"""

import math


class InductionMotorCalculator:
    """Core calculation engine for induction motor analysis"""

    def __init__(self, V_line=400, frequency=50, poles=4, R2=0.01, X2=0.1, a=4):
        """
        Initialize motor parameters

        Parameters:
        -----------
        V_line : float
            Line voltage (V)
        frequency : float
            Supply frequency (Hz)
        poles : int
            Number of poles
        R2 : float
            Rotor resistance per phase (Ohm)
        X2 : float
            Rotor reactance per phase (Ohm)
        a : float
            Stator to rotor turns ratio
        """
        self.V_line = V_line
        self.frequency = frequency
        self.poles = poles
        self.R2 = R2
        self.X2 = X2
        self.a = a

    def calculate_phase_voltage(self):
        """Calculate phase voltage (assuming star connection)"""
        return self.V_line / math.sqrt(3)

    def calculate_sync_speed(self):
        """Calculate synchronous speed in RPM"""
        return (120 * self.frequency) / self.poles

    def calculate_omega_sync(self):
        """Calculate synchronous angular velocity (rad/s)"""
        return (4 * math.pi * self.frequency) / self.poles

    def calculate_max_torque_slip(self):
        """
        Calculate maximum torque and corresponding slip

        For an induction motor:
        - Slip at maximum torque: s_max = R2/X2
        - Maximum torque: T_max = (3 * V_ph^2) / (2 * ω_s * X2)

        Returns:
        --------
        T_max : float
            Maximum torque (N·m)
        s_max : float
            Slip at maximum torque
        """
        # Slip at maximum torque
        s_max = self.R2 / self.X2

        # Synchronous angular velocity
        omega_s = self.calculate_omega_sync()

        # Phase voltage
        V_ph = self.calculate_phase_voltage()

        # Maximum torque (simplified formula)
        T_max = (3 * V_ph**2) / (2 * omega_s * self.X2)

        return T_max, s_max

    def calculate_full_load_parameters(self):
        """
        Calculate full load slip and power output
        Given: Maximum torque is twice the full load torque (T_max = 2 * T_fl)

        Using torque-slip relationship:
        T/T_max = 2 / (s/s_max + s_max/s)

        For T_fl = T_max/2:
        1/2 = 2 / (s/s_max + s_max/s)
        s/s_max + s_max/s = 4

        Let x = s/s_max:
        x + 1/x = 4
        x^2 - 4x + 1 = 0
        x = (4 ± √12)/2 = 2 ± √3

        We take x = 2 - √3 (stable operating region)

        Returns:
        --------
        T_fl : float
            Full load torque (N·m)
        s_fl : float
            Full load slip
        P_out : float
            Full load power output (W)
        omega_r : float
            Rotor angular velocity (rad/s)
        N_fl : float
            Full load speed (RPM)
        """
        # Get maximum torque and slip
        T_max, s_max = self.calculate_max_torque_slip()

        # Full load torque (given: T_max = 2 * T_fl)
        T_fl = T_max / 2

        # Calculate full load slip
        # From equation: x^2 - 4x + 1 = 0, where x = s_fl/s_max
        # x = 2 - √3 (taking the smaller root for stable operation)
        x = 2 - math.sqrt(3)
        s_fl = x * s_max

        # Synchronous angular velocity
        omega_s = self.calculate_omega_sync()

        # Rotor angular velocity at full load
        omega_r = omega_s * (1 - s_fl)

        # Full load power output
        P_out = T_fl * omega_r

        # Synchronous and full load speed in RPM
        N_sync = self.calculate_sync_speed()
        N_fl = N_sync * (1 - s_fl)

        return T_fl, s_fl, P_out, omega_r, N_fl

    def calculate_torque_at_slip(self, s):
        """
        Calculate torque at a given slip value

        Parameters:
        -----------
        s : float
            Slip value

        Returns:
        --------
        T : float
            Torque at given slip (N·m)
        """
        if s <= 0:
            return 0

        omega_s = self.calculate_omega_sync()
        V_ph = self.calculate_phase_voltage()

        # Using simplified torque equation
        # Assuming R1 << R2/s (stator resistance neglected)
        R2_s = self.R2 / s
        Z = math.sqrt(R2_s**2 + self.X2**2)
        I2 = V_ph / Z

        # Torque
        T = (3 * I2**2 * R2_s) / (omega_s * s)

        return T

    def print_detailed_solution(self):
        """Print detailed solution to Question 5"""
        print("=" * 80)
        print(" " * 20 + "INDUCTION MOTOR ANALYSIS - QUESTION 5")
        print("=" * 80)
        print()

        # Print given parameters
        print("GIVEN PARAMETERS:")
        print("-" * 80)
        print(f"  Line Voltage (V_L)              : {self.V_line} V")
        print(f"  Phase Voltage (V_ph)            : {self.calculate_phase_voltage():.2f} V (Star connection)")
        print(f"  Frequency (f)                   : {self.frequency} Hz")
        print(f"  Number of Poles (P)             : {self.poles}")
        print(f"  Number of Phases                : 3")
        print(f"  Rotor Resistance per phase (R2) : {self.R2} Ω")
        print(f"  Rotor Reactance per phase (X2)  : {self.X2} Ω")
        print(f"  Stator to Rotor Turns Ratio (a) : {self.a}")
        print()

        # Calculate basic parameters
        N_sync = self.calculate_sync_speed()
        omega_s = self.calculate_omega_sync()
        V_ph = self.calculate_phase_voltage()

        print("BASIC CALCULATIONS:")
        print("-" * 80)
        print(f"  Synchronous Speed (N_s)         : N_s = (120 × f) / P")
        print(f"                                  : N_s = (120 × {self.frequency}) / {self.poles}")
        print(f"                                  : N_s = {N_sync:.2f} RPM")
        print()
        print(f"  Synchronous Angular Velocity    : ω_s = (4π × f) / P")
        print(f"                                  : ω_s = (4π × {self.frequency}) / {self.poles}")
        print(f"                                  : ω_s = {omega_s:.4f} rad/s")
        print()

        # Part (i): Maximum torque and slip
        print("=" * 80)
        print("(i) MAXIMUM TORQUE AND CORRESPONDING SLIP:")
        print("=" * 80)

        T_max, s_max = self.calculate_max_torque_slip()

        print()
        print("  Formula for slip at maximum torque:")
        print("    s_max = R2 / X2")
        print(f"    s_max = {self.R2} / {self.X2}")
        print(f"    s_max = {s_max:.4f} or {s_max * 100:.2f}%")
        print()

        print("  Formula for maximum torque:")
        print("    T_max = (3 × V_ph²) / (2 × ω_s × X2)")
        print(f"    T_max = (3 × {V_ph:.2f}²) / (2 × {omega_s:.4f} × {self.X2})")
        print(f"    T_max = (3 × {V_ph**2:.2f}) / {2 * omega_s * self.X2:.4f}")
        print(f"    T_max = {T_max:.2f} N·m")
        print()

        print(f"  ANSWER (i):")
        print(f"    Maximum Torque (T_max)          : {T_max:.2f} N·m")
        print(f"    Slip at Maximum Torque (s_max)  : {s_max:.4f} ({s_max * 100:.2f}%)")
        print()

        # Part (ii): Full load parameters
        print("=" * 80)
        print("(ii) FULL LOAD SLIP AND POWER OUTPUT:")
        print("=" * 80)
        print()
        print("  Given: Maximum torque is twice the full load torque")
        print("         T_max = 2 × T_fl")
        print(f"         Therefore: T_fl = T_max / 2 = {T_max:.2f} / 2 = {T_max/2:.2f} N·m")
        print()

        print("  Using torque-slip relationship:")
        print("    T/T_max = 2 / (s/s_max + s_max/s)")
        print()
        print("  For full load: T_fl/T_max = 1/2")
        print("    1/2 = 2 / (s_fl/s_max + s_max/s_fl)")
        print("    s_fl/s_max + s_max/s_fl = 4")
        print()
        print("  Let x = s_fl/s_max:")
        print("    x + 1/x = 4")
        print("    x² - 4x + 1 = 0")
        print()
        print("  Using quadratic formula:")
        print("    x = (4 ± √(16-4)) / 2")
        print("    x = (4 ± √12) / 2")
        print("    x = (4 ± 2√3) / 2")
        print("    x = 2 ± √3")
        print()
        print("  Two solutions:")
        print(f"    x₁ = 2 + √3 = {2 + math.sqrt(3):.6f} (unstable region)")
        print(f"    x₂ = 2 - √3 = {2 - math.sqrt(3):.6f} (stable region)")
        print()
        print("  Taking stable operating point: x = 2 - √3")
        print()

        T_fl, s_fl, P_out, omega_r, N_fl = self.calculate_full_load_parameters()

        print(f"  Full Load Slip:")
        print(f"    s_fl = x × s_max")
        print(f"    s_fl = {2 - math.sqrt(3):.6f} × {s_max:.4f}")
        print(f"    s_fl = {s_fl:.6f} or {s_fl * 100:.4f}%")
        print()

        print(f"  Full Load Speed:")
        print(f"    N_fl = N_s × (1 - s_fl)")
        print(f"    N_fl = {N_sync:.2f} × (1 - {s_fl:.6f})")
        print(f"    N_fl = {N_fl:.2f} RPM")
        print()

        print(f"  Rotor Angular Velocity:")
        print(f"    ω_r = ω_s × (1 - s_fl)")
        print(f"    ω_r = {omega_s:.4f} × (1 - {s_fl:.6f})")
        print(f"    ω_r = {omega_r:.4f} rad/s")
        print()

        print(f"  Full Load Power Output:")
        print(f"    P_out = T_fl × ω_r")
        print(f"    P_out = {T_fl:.2f} × {omega_r:.4f}")
        print(f"    P_out = {P_out:.2f} W")
        print(f"    P_out = {P_out/1000:.3f} kW")
        print()

        print(f"  ANSWER (ii):")
        print(f"    Full Load Torque (T_fl)         : {T_fl:.2f} N·m")
        print(f"    Full Load Slip (s_fl)           : {s_fl:.6f} ({s_fl * 100:.4f}%)")
        print(f"    Full Load Speed (N_fl)          : {N_fl:.2f} RPM")
        print(f"    Full Load Power Output (P_out)  : {P_out:.2f} W ({P_out/1000:.3f} kW)")
        print()

        # Verification
        print("=" * 80)
        print("VERIFICATION:")
        print("=" * 80)
        print(f"  T_max / T_fl ratio              : {T_max/T_fl:.6f} ≈ 2.0 ✓")
        print(f"  Speed regulation                : {((N_sync - N_fl) / N_sync) * 100:.4f}%")
        print()

        # Additional information
        print("=" * 80)
        print("ADDITIONAL ANALYSIS:")
        print("=" * 80)

        # Calculate some additional points
        slips = [0.01, 0.05, s_fl, s_max, 0.2, 0.5, 1.0]
        print()
        print("  Torque at various slip values:")
        print("  " + "-" * 76)
        print(f"  {'Slip':<10} {'Slip (%)':<12} {'Speed (RPM)':<15} {'Torque (N·m)':<15}")
        print("  " + "-" * 76)

        for s in slips:
            T = self.calculate_torque_at_slip(s)
            N = N_sync * (1 - s)
            marker = ""
            if abs(s - s_fl) < 0.0001:
                marker = " ← Full Load"
            elif abs(s - s_max) < 0.0001:
                marker = " ← Maximum Torque"
            print(f"  {s:<10.6f} {s*100:<12.4f} {N:<15.2f} {T:<15.2f}{marker}")

        print("  " + "-" * 76)
        print()

        print("=" * 80)
        print(" " * 25 + "END OF ANALYSIS")
        print("=" * 80)


def main():
    """Main function"""
    print("\n")

    # Create motor instance with Question 5 parameters
    motor = InductionMotorCalculator(
        V_line=400,      # Line voltage (V)
        frequency=50,    # Frequency (Hz)
        poles=4,         # Number of poles
        R2=0.01,         # Rotor resistance (Ω)
        X2=0.1,          # Rotor reactance (Ω)
        a=4              # Turns ratio
    )

    # Print detailed solution
    motor.print_detailed_solution()

    print("\n" + "=" * 80)
    print("SUMMARY OF ANSWERS:")
    print("=" * 80)

    T_max, s_max = motor.calculate_max_torque_slip()
    T_fl, s_fl, P_out, omega_r, N_fl = motor.calculate_full_load_parameters()

    print()
    print(f"(i)  Maximum Torque:           T_max = {T_max:.2f} N·m")
    print(f"     Slip at Maximum Torque:  s_max = {s_max:.4f} ({s_max*100:.2f}%)")
    print()
    print(f"(ii) Full Load Torque:         T_fl  = {T_fl:.2f} N·m")
    print(f"     Full Load Slip:           s_fl  = {s_fl:.6f} ({s_fl*100:.4f}%)")
    print(f"     Full Load Power Output:   P_out = {P_out:.2f} W ({P_out/1000:.3f} kW)")
    print(f"     Full Load Speed:          N_fl  = {N_fl:.2f} RPM")
    print()
    print("=" * 80)
    print("\n")


if __name__ == "__main__":
    main()
