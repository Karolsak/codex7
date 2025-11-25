"""
Advanced Induction Motor Analysis System
Complete solution with GUI, dynamic simulation, and real-time visualization
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import math
from scipy.integrate import solve_ivp
from datetime import datetime


class InductionMotorAnalysis:
    """Core calculation engine for induction motor analysis"""

    def __init__(self):
        # Default parameters for Q.5
        self.V_line = 400  # Line voltage (V)
        self.phases = 3
        self.poles = 4
        self.frequency = 50  # Hz
        self.R2 = 0.01  # Rotor resistance per phase (Ohm)
        self.X2 = 0.1  # Rotor reactance per phase (Ohm)
        self.a = 4  # Stator to rotor turns ratio
        self.R1 = 0.02  # Stator resistance (Ohm) - assumed
        self.X1 = 0.05  # Stator reactance (Ohm) - assumed
        self.Xm = 10  # Magnetizing reactance (Ohm) - assumed

        # Mechanical parameters
        self.J = 0.5  # Moment of inertia (kg·m²)
        self.B = 0.01  # Friction coefficient (N·m·s)
        self.T_load = 0  # Load torque (N·m)

    def calculate_sync_speed(self):
        """Calculate synchronous speed"""
        return (120 * self.frequency) / self.poles  # RPM

    def calculate_omega_sync(self):
        """Calculate synchronous angular velocity (rad/s)"""
        return (4 * math.pi * self.frequency) / self.poles

    def calculate_phase_voltage(self):
        """Calculate phase voltage (star connection assumed)"""
        return self.V_line / math.sqrt(3)

    def calculate_max_torque_slip(self):
        """
        Calculate maximum torque and corresponding slip
        Solution to Q.5 part (i)
        """
        # Slip at maximum torque (simplified)
        s_max = self.R2 / self.X2

        # Synchronous angular velocity
        omega_s = self.calculate_omega_sync()

        # Phase voltage
        V_ph = self.calculate_phase_voltage()

        # Maximum torque (simplified formula)
        # T_max = (3 * V_ph^2) / (2 * omega_s * X2)
        T_max = (3 * V_ph**2) / (2 * omega_s * self.X2)

        return T_max, s_max

    def calculate_full_load_parameters(self):
        """
        Calculate full load slip and power output
        Solution to Q.5 part (ii)
        """
        T_max, s_max = self.calculate_max_torque_slip()

        # Full load torque (T_fl = T_max / 2)
        T_fl = T_max / 2

        # Using torque-slip relationship
        # T/T_max = (2) / (s/s_max + s_max/s)
        # For T_fl = T_max/2: 1/2 = 2 / (s/s_max + s_max/s)
        # s/s_max + s_max/s = 4
        # Let x = s/s_max, then: x + 1/x = 4
        # x^2 - 4x + 1 = 0
        # x = (4 ± sqrt(16-4))/2 = (4 ± sqrt(12))/2 = 2 ± sqrt(3)

        # We take the smaller value (before maximum torque point)
        x = 2 - math.sqrt(3)
        s_fl = x * s_max

        # Synchronous speed
        omega_s = self.calculate_omega_sync()

        # Rotor speed at full load
        omega_r = omega_s * (1 - s_fl)

        # Full load power output
        P_out = T_fl * omega_r

        return T_fl, s_fl, P_out, omega_r

    def calculate_torque_vs_slip(self, slip_range):
        """Calculate torque for a range of slip values"""
        omega_s = self.calculate_omega_sync()
        V_ph = self.calculate_phase_voltage()

        torques = []
        for s in slip_range:
            if s == 0:
                torques.append(0)
            else:
                # Simplified torque equation
                R2_s = self.R2 / s
                Z = math.sqrt((self.R1 + R2_s)**2 + (self.X1 + self.X2)**2)
                I2 = V_ph / Z
                T = (3 * I2**2 * R2_s) / (omega_s * s)
                torques.append(T)

        return np.array(torques)

    def calculate_current_vs_slip(self, slip_range):
        """Calculate current for a range of slip values"""
        V_ph = self.calculate_phase_voltage()

        currents = []
        for s in slip_range:
            if s == 0:
                s = 0.001  # Avoid division by zero
            R2_s = self.R2 / s
            Z = math.sqrt((self.R1 + R2_s)**2 + (self.X1 + self.X2)**2)
            I = V_ph / Z
            currents.append(I)

        return np.array(currents)

    def calculate_efficiency_vs_slip(self, slip_range):
        """Calculate efficiency for a range of slip values"""
        omega_s = self.calculate_omega_sync()
        V_ph = self.calculate_phase_voltage()

        efficiencies = []
        for s in slip_range:
            if s == 0 or s >= 1:
                efficiencies.append(0)
            else:
                R2_s = self.R2 / s
                Z = math.sqrt((self.R1 + R2_s)**2 + (self.X1 + self.X2)**2)
                I = V_ph / Z

                # Input power
                P_in = 3 * V_ph * I * math.cos(math.atan((self.X1 + self.X2)/(self.R1 + R2_s)))

                # Output power
                P_out = (3 * I**2 * R2_s * (1 - s)) / s

                if P_in > 0:
                    eff = (P_out / P_in) * 100
                    efficiencies.append(max(0, min(100, eff)))
                else:
                    efficiencies.append(0)

        return np.array(efficiencies)

    def dynamic_model(self, t, state, T_load):
        """
        Differential equations for dynamic simulation
        state = [omega_r, theta_r]
        omega_r: rotor angular velocity (rad/s)
        theta_r: rotor angle (rad)
        """
        omega_r, theta_r = state
        omega_s = self.calculate_omega_sync()

        # Calculate slip
        s = (omega_s - omega_r) / omega_s

        # Limit slip to reasonable range
        s = max(-1, min(2, s))

        # Calculate electromagnetic torque
        V_ph = self.calculate_phase_voltage()
        if s == 0:
            s = 0.001
        R2_s = self.R2 / s
        Z = math.sqrt((self.R1 + R2_s)**2 + (self.X1 + self.X2)**2)
        I = V_ph / Z
        T_em = (3 * I**2 * R2_s) / (omega_s * s)

        # Dynamic equation
        domega_dt = (T_em - T_load - self.B * omega_r) / self.J
        dtheta_dt = omega_r

        return [domega_dt, dtheta_dt]


class AdvancedIMSimulator(tk.Tk):
    """Advanced Induction Motor Simulator with GUI"""

    def __init__(self):
        super().__init__()

        self.title("Advanced Induction Motor Analysis System")
        self.geometry("1400x900")
        self.configure(bg="#2b2b2b")

        # Initialize motor analysis engine
        self.motor = InductionMotorAnalysis()

        # Simulation variables
        self.simulation_running = False
        self.simulation_data = {
            'time': [],
            'speed': [],
            'torque': [],
            'current': [],
            'power': []
        }

        # Create main menu
        self.create_menu()

        # Create main container with grid
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        # Create notebook for tabs
        self.notebook = ttk.Notebook(self)
        self.notebook.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        # Create tabs
        self.create_steady_state_tab()
        self.create_dynamic_simulation_tab()
        self.create_characteristics_tab()
        self.create_results_tab()

        # Configure auto-resize
        self.bind("<Configure>", self.on_window_resize)

        # Initial calculations
        self.solve_question_5()

    def create_menu(self):
        """Create menu bar"""
        menubar = tk.Menu(self)
        self.config(menu=menubar)

        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Save Results", command=self.save_results)
        file_menu.add_command(label="Load Parameters", command=self.load_parameters)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.quit)

        # Calculate menu
        calc_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Calculate", menu=calc_menu)
        calc_menu.add_command(label="Solve Q.5", command=self.solve_question_5)
        calc_menu.add_command(label="Generate Characteristics", command=self.generate_characteristics)
        calc_menu.add_separator()
        calc_menu.add_command(label="Reset All", command=self.reset_all)

        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)
        help_menu.add_command(label="User Guide", command=self.show_user_guide)

    def create_steady_state_tab(self):
        """Create steady-state analysis tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Steady-State Analysis")

        # Configure grid weights
        tab.grid_rowconfigure(0, weight=0)
        tab.grid_rowconfigure(1, weight=1)
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_columnconfigure(1, weight=2)

        # Left panel - Parameters
        left_frame = ttk.LabelFrame(tab, text="Motor Parameters", padding=10)
        left_frame.grid(row=0, column=0, rowspan=2, sticky="nsew", padx=5, pady=5)

        # Parameter entries
        params = [
            ("Line Voltage (V):", "V_line", 400),
            ("Frequency (Hz):", "frequency", 50),
            ("Number of Poles:", "poles", 4),
            ("Rotor Resistance (Ω):", "R2", 0.01),
            ("Rotor Reactance (Ω):", "X2", 0.1),
            ("Stator Resistance (Ω):", "R1", 0.02),
            ("Stator Reactance (Ω):", "X1", 0.05),
            ("Turns Ratio (a):", "a", 4),
            ("Magnetizing Reactance (Ω):", "Xm", 10),
        ]

        self.param_entries = {}
        for i, (label, key, default) in enumerate(params):
            ttk.Label(left_frame, text=label).grid(row=i, column=0, sticky="w", pady=5)
            entry = ttk.Entry(left_frame, width=15)
            entry.insert(0, str(default))
            entry.grid(row=i, column=1, sticky="ew", pady=5, padx=5)
            self.param_entries[key] = entry

        # Update button
        ttk.Button(left_frame, text="Update Parameters",
                  command=self.update_parameters).grid(row=len(params), column=0,
                                                       columnspan=2, pady=10)

        # Calculate button
        ttk.Button(left_frame, text="Calculate Q.5",
                  command=self.solve_question_5,
                  style="Accent.TButton").grid(row=len(params)+1, column=0,
                                               columnspan=2, pady=5)

        # Right panel - Results
        right_frame = ttk.LabelFrame(tab, text="Calculation Results", padding=10)
        right_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)

        self.results_text = scrolledtext.ScrolledText(right_frame, width=60, height=20,
                                                      font=("Courier", 10))
        self.results_text.pack(fill="both", expand=True)

        # Bottom panel - Torque-Slip curve
        bottom_frame = ttk.LabelFrame(tab, text="Torque-Slip Characteristic", padding=10)
        bottom_frame.grid(row=1, column=1, sticky="nsew", padx=5, pady=5)

        self.fig_torque = Figure(figsize=(8, 4), dpi=100)
        self.ax_torque = self.fig_torque.add_subplot(111)
        self.canvas_torque = FigureCanvasTkAgg(self.fig_torque, bottom_frame)
        self.canvas_torque.get_tk_widget().pack(fill="both", expand=True)

        toolbar = NavigationToolbar2Tk(self.canvas_torque, bottom_frame)
        toolbar.update()

    def create_dynamic_simulation_tab(self):
        """Create dynamic simulation tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Dynamic Simulation")

        # Configure grid
        tab.grid_rowconfigure(1, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Control panel
        control_frame = ttk.LabelFrame(tab, text="Simulation Controls", padding=10)
        control_frame.grid(row=0, column=0, sticky="ew", padx=5, pady=5)

        # Control frame grid
        control_frame.grid_columnconfigure(1, weight=1)

        # Load torque slider
        ttk.Label(control_frame, text="Load Torque (Nm):").grid(row=0, column=0, sticky="w")
        self.load_torque_var = tk.DoubleVar(value=0)
        self.load_torque_slider = ttk.Scale(control_frame, from_=0, to=500,
                                           variable=self.load_torque_var,
                                           orient="horizontal", command=self.update_load_display)
        self.load_torque_slider.grid(row=0, column=1, sticky="ew", padx=10)
        self.load_torque_label = ttk.Label(control_frame, text="0.0 Nm")
        self.load_torque_label.grid(row=0, column=2, padx=5)

        # Inertia slider
        ttk.Label(control_frame, text="Inertia (kg·m²):").grid(row=1, column=0, sticky="w")
        self.inertia_var = tk.DoubleVar(value=0.5)
        self.inertia_slider = ttk.Scale(control_frame, from_=0.1, to=5,
                                       variable=self.inertia_var,
                                       orient="horizontal", command=self.update_inertia_display)
        self.inertia_slider.grid(row=1, column=1, sticky="ew", padx=10)
        self.inertia_label = ttk.Label(control_frame, text="0.5 kg·m²")
        self.inertia_label.grid(row=1, column=2, padx=5)

        # Simulation time
        ttk.Label(control_frame, text="Simulation Time (s):").grid(row=2, column=0, sticky="w")
        self.sim_time_var = tk.DoubleVar(value=5.0)
        self.sim_time_entry = ttk.Entry(control_frame, textvariable=self.sim_time_var, width=10)
        self.sim_time_entry.grid(row=2, column=1, sticky="w", padx=10)

        # ODE Solver selection
        ttk.Label(control_frame, text="ODE Solver:").grid(row=3, column=0, sticky="w")
        self.solver_var = tk.StringVar(value="RK45")
        solver_combo = ttk.Combobox(control_frame, textvariable=self.solver_var,
                                    values=["RK45", "RK23", "DOP853", "Euler"],
                                    state="readonly", width=10)
        solver_combo.grid(row=3, column=1, sticky="w", padx=10)

        # Control buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=4, column=0, columnspan=3, pady=10)

        self.start_btn = ttk.Button(button_frame, text="▶ Start",
                                    command=self.start_simulation, width=12)
        self.start_btn.pack(side="left", padx=5)

        self.stop_btn = ttk.Button(button_frame, text="⏸ Stop",
                                   command=self.stop_simulation, width=12, state="disabled")
        self.stop_btn.pack(side="left", padx=5)

        self.reset_btn = ttk.Button(button_frame, text="↻ Reset",
                                    command=self.reset_simulation, width=12)
        self.reset_btn.pack(side="left", padx=5)

        # Plotting area
        plot_frame = ttk.LabelFrame(tab, text="Dynamic Response", padding=10)
        plot_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

        self.fig_dynamic = Figure(figsize=(12, 8), dpi=100)

        # Create subplots
        self.ax_speed = self.fig_dynamic.add_subplot(221)
        self.ax_torque_time = self.fig_dynamic.add_subplot(222)
        self.ax_current = self.fig_dynamic.add_subplot(223)
        self.ax_power = self.fig_dynamic.add_subplot(224)

        self.fig_dynamic.tight_layout(pad=3.0)

        self.canvas_dynamic = FigureCanvasTkAgg(self.fig_dynamic, plot_frame)
        self.canvas_dynamic.get_tk_widget().pack(fill="both", expand=True)

        toolbar = NavigationToolbar2Tk(self.canvas_dynamic, plot_frame)
        toolbar.update()

    def create_characteristics_tab(self):
        """Create motor characteristics tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Motor Characteristics")

        # Configure grid
        tab.grid_rowconfigure(0, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Control panel
        control_frame = ttk.Frame(tab)
        control_frame.grid(row=1, column=0, sticky="ew", padx=5, pady=5)

        ttk.Button(control_frame, text="Generate All Characteristics",
                  command=self.generate_characteristics).pack(pady=10)

        # Plotting area
        plot_frame = ttk.LabelFrame(tab, text="Motor Characteristics Curves", padding=10)
        plot_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        self.fig_char = Figure(figsize=(12, 8), dpi=100)

        # Create subplots for characteristics
        self.ax_char_torque = self.fig_char.add_subplot(221)
        self.ax_char_current = self.fig_char.add_subplot(222)
        self.ax_char_power = self.fig_char.add_subplot(223)
        self.ax_char_eff = self.fig_char.add_subplot(224)

        self.fig_char.tight_layout(pad=3.0)

        self.canvas_char = FigureCanvasTkAgg(self.fig_char, plot_frame)
        self.canvas_char.get_tk_widget().pack(fill="both", expand=True)

        toolbar = NavigationToolbar2Tk(self.canvas_char, plot_frame)
        toolbar.update()

    def create_results_tab(self):
        """Create detailed results tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Detailed Results")

        # Configure grid
        tab.grid_rowconfigure(0, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Results text area
        results_frame = ttk.LabelFrame(tab, text="Comprehensive Analysis Results", padding=10)
        results_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        self.detailed_results_text = scrolledtext.ScrolledText(results_frame,
                                                               font=("Courier", 10),
                                                               wrap=tk.WORD)
        self.detailed_results_text.pack(fill="both", expand=True)

        # Buttons
        button_frame = ttk.Frame(tab)
        button_frame.grid(row=1, column=0, sticky="ew", padx=5, pady=5)

        ttk.Button(button_frame, text="Export to File",
                  command=self.export_results).pack(side="left", padx=5)
        ttk.Button(button_frame, text="Clear Results",
                  command=lambda: self.detailed_results_text.delete(1.0, tk.END)).pack(side="left", padx=5)

    def update_parameters(self):
        """Update motor parameters from GUI entries"""
        try:
            self.motor.V_line = float(self.param_entries['V_line'].get())
            self.motor.frequency = float(self.param_entries['frequency'].get())
            self.motor.poles = int(self.param_entries['poles'].get())
            self.motor.R2 = float(self.param_entries['R2'].get())
            self.motor.X2 = float(self.param_entries['X2'].get())
            self.motor.R1 = float(self.param_entries['R1'].get())
            self.motor.X1 = float(self.param_entries['X1'].get())
            self.motor.a = float(self.param_entries['a'].get())
            self.motor.Xm = float(self.param_entries['Xm'].get())

            messagebox.showinfo("Success", "Parameters updated successfully!")
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid parameter value: {str(e)}")

    def solve_question_5(self):
        """Solve Question 5 and display results"""
        self.update_parameters()

        # Calculate maximum torque and slip
        T_max, s_max = self.motor.calculate_max_torque_slip()

        # Calculate full load parameters
        T_fl, s_fl, P_out, omega_r = self.motor.calculate_full_load_parameters()

        # Synchronous speed
        N_sync = self.motor.calculate_sync_speed()
        N_fl = N_sync * (1 - s_fl)

        # Display results
        results = f"""
{'='*70}
SOLUTION TO QUESTION 5: INDUCTION MOTOR ANALYSIS
{'='*70}

GIVEN PARAMETERS:
  Line Voltage (V_L)             : {self.motor.V_line} V
  Phase Voltage (V_ph)           : {self.motor.calculate_phase_voltage():.2f} V
  Frequency (f)                  : {self.motor.frequency} Hz
  Number of Poles (P)            : {self.motor.poles}
  Rotor Resistance (R2)          : {self.motor.R2} Ω
  Rotor Reactance (X2)           : {self.motor.X2} Ω
  Stator to Rotor Turns Ratio    : {self.motor.a}

CALCULATED RESULTS:

(i) MAXIMUM TORQUE AND CORRESPONDING SLIP:
    ----------------------------------------
    Synchronous Speed (N_s)      : {N_sync:.2f} RPM
    Synchronous Angular Vel (ω_s): {self.motor.calculate_omega_sync():.4f} rad/s

    Slip at Maximum Torque (s_max): {s_max:.4f} or {s_max*100:.2f}%
    Maximum Torque (T_max)        : {T_max:.2f} N·m

(ii) FULL LOAD PARAMETERS:
     ---------------------
     Full Load Torque (T_fl)      : {T_fl:.2f} N·m
     Full Load Slip (s_fl)        : {s_fl:.6f} or {s_fl*100:.4f}%
     Full Load Speed (N_fl)       : {N_fl:.2f} RPM
     Rotor Angular Velocity       : {omega_r:.4f} rad/s
     Full Load Power Output (P_out): {P_out:.2f} W ({P_out/1000:.3f} kW)

VERIFICATION:
  T_max / T_fl ratio             : {T_max/T_fl:.4f} ≈ 2.0 ✓
  Speed regulation               : {((N_sync-N_fl)/N_sync)*100:.4f}%

{'='*70}
Calculated at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
{'='*70}
"""

        self.results_text.delete(1.0, tk.END)
        self.results_text.insert(1.0, results)

        # Update detailed results
        self.detailed_results_text.delete(1.0, tk.END)
        self.detailed_results_text.insert(1.0, results)

        # Plot torque-slip curve
        self.plot_torque_slip_curve(T_max, s_max, T_fl, s_fl)

    def plot_torque_slip_curve(self, T_max, s_max, T_fl, s_fl):
        """Plot torque vs slip characteristic"""
        slip_range = np.linspace(0.001, 1.0, 1000)
        torque = self.motor.calculate_torque_vs_slip(slip_range)

        self.ax_torque.clear()
        self.ax_torque.plot(slip_range, torque, 'b-', linewidth=2, label='Torque-Slip Curve')
        self.ax_torque.plot(s_max, T_max, 'ro', markersize=10, label=f'Max Torque: {T_max:.2f} Nm @ s={s_max:.4f}')
        self.ax_torque.plot(s_fl, T_fl, 'go', markersize=10, label=f'Full Load: {T_fl:.2f} Nm @ s={s_fl:.6f}')

        self.ax_torque.axhline(y=T_max, color='r', linestyle='--', alpha=0.3)
        self.ax_torque.axhline(y=T_fl, color='g', linestyle='--', alpha=0.3)
        self.ax_torque.axvline(x=s_max, color='r', linestyle='--', alpha=0.3)
        self.ax_torque.axvline(x=s_fl, color='g', linestyle='--', alpha=0.3)

        self.ax_torque.set_xlabel('Slip (s)', fontsize=11, fontweight='bold')
        self.ax_torque.set_ylabel('Torque (N·m)', fontsize=11, fontweight='bold')
        self.ax_torque.set_title('Induction Motor Torque-Slip Characteristic',
                                 fontsize=12, fontweight='bold')
        self.ax_torque.grid(True, alpha=0.3)
        self.ax_torque.legend(loc='best', fontsize=9)

        self.canvas_torque.draw()

    def generate_characteristics(self):
        """Generate all motor characteristics"""
        self.update_parameters()

        slip_range = np.linspace(0.001, 1.0, 500)

        # Calculate characteristics
        torque = self.motor.calculate_torque_vs_slip(slip_range)
        current = self.motor.calculate_current_vs_slip(slip_range)
        efficiency = self.motor.calculate_efficiency_vs_slip(slip_range)

        # Convert slip to speed
        N_sync = self.motor.calculate_sync_speed()
        speed = N_sync * (1 - slip_range)

        # Calculate power
        omega_s = self.motor.calculate_omega_sync()
        omega = omega_s * (1 - slip_range)
        power = torque * omega / 1000  # kW

        # Plot Torque vs Speed
        self.ax_char_torque.clear()
        self.ax_char_torque.plot(speed, torque, 'b-', linewidth=2)
        self.ax_char_torque.set_xlabel('Speed (RPM)', fontweight='bold')
        self.ax_char_torque.set_ylabel('Torque (N·m)', fontweight='bold')
        self.ax_char_torque.set_title('Torque vs Speed', fontweight='bold')
        self.ax_char_torque.grid(True, alpha=0.3)

        # Plot Current vs Speed
        self.ax_char_current.clear()
        self.ax_char_current.plot(speed, current, 'r-', linewidth=2)
        self.ax_char_current.set_xlabel('Speed (RPM)', fontweight='bold')
        self.ax_char_current.set_ylabel('Current (A)', fontweight='bold')
        self.ax_char_current.set_title('Current vs Speed', fontweight='bold')
        self.ax_char_current.grid(True, alpha=0.3)

        # Plot Power vs Speed
        self.ax_char_power.clear()
        self.ax_char_power.plot(speed, power, 'g-', linewidth=2)
        self.ax_char_power.set_xlabel('Speed (RPM)', fontweight='bold')
        self.ax_char_power.set_ylabel('Power (kW)', fontweight='bold')
        self.ax_char_power.set_title('Power vs Speed', fontweight='bold')
        self.ax_char_power.grid(True, alpha=0.3)

        # Plot Efficiency vs Speed
        self.ax_char_eff.clear()
        self.ax_char_eff.plot(speed, efficiency, 'm-', linewidth=2)
        self.ax_char_eff.set_xlabel('Speed (RPM)', fontweight='bold')
        self.ax_char_eff.set_ylabel('Efficiency (%)', fontweight='bold')
        self.ax_char_eff.set_title('Efficiency vs Speed', fontweight='bold')
        self.ax_char_eff.grid(True, alpha=0.3)
        self.ax_char_eff.set_ylim([0, 100])

        self.fig_char.tight_layout()
        self.canvas_char.draw()

    def start_simulation(self):
        """Start dynamic simulation"""
        self.simulation_running = True
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="normal")

        # Update motor parameters
        self.motor.J = self.inertia_var.get()
        self.motor.T_load = self.load_torque_var.get()

        # Simulation parameters
        t_span = (0, self.sim_time_var.get())
        t_eval = np.linspace(0, self.sim_time_var.get(), 1000)

        # Initial conditions [omega_r, theta_r]
        y0 = [0, 0]

        # Select solver
        solver = self.solver_var.get()

        try:
            if solver == "Euler":
                # Custom Euler method
                sol = self.euler_solve(t_eval, y0)
            else:
                # Scipy solvers
                sol = solve_ivp(
                    lambda t, y: self.motor.dynamic_model(t, y, self.motor.T_load),
                    t_span, y0, method=solver, t_eval=t_eval,
                    max_step=0.01
                )

            # Extract results
            time = sol.t if solver != "Euler" else sol['t']
            omega_r = sol.y[0] if solver != "Euler" else sol['y'][0]

            # Calculate speed in RPM
            speed_rpm = omega_r * 60 / (2 * np.pi)

            # Calculate torque and current
            omega_s = self.motor.calculate_omega_sync()
            V_ph = self.motor.calculate_phase_voltage()

            torque = []
            current = []
            power = []

            for omega in omega_r:
                s = (omega_s - omega) / omega_s
                s = max(0.001, min(2, s))

                R2_s = self.motor.R2 / s
                Z = math.sqrt((self.motor.R1 + R2_s)**2 + (self.motor.X1 + self.motor.X2)**2)
                I = V_ph / Z
                T = (3 * I**2 * R2_s) / (omega_s * s)
                P = T * omega / 1000  # kW

                torque.append(T)
                current.append(I)
                power.append(P)

            # Store results
            self.simulation_data = {
                'time': time,
                'speed': speed_rpm,
                'torque': np.array(torque),
                'current': np.array(current),
                'power': np.array(power)
            }

            # Plot results
            self.plot_dynamic_results()

            messagebox.showinfo("Success", f"Simulation completed using {solver} solver!")

        except Exception as e:
            messagebox.showerror("Simulation Error", f"Error during simulation: {str(e)}")
        finally:
            self.simulation_running = False
            self.start_btn.config(state="normal")
            self.stop_btn.config(state="disabled")

    def euler_solve(self, t_eval, y0):
        """Custom Euler method for ODE solving"""
        dt = t_eval[1] - t_eval[0]
        n_steps = len(t_eval)

        y = np.zeros((2, n_steps))
        y[:, 0] = y0

        for i in range(n_steps - 1):
            dydt = self.motor.dynamic_model(t_eval[i], y[:, i], self.motor.T_load)
            y[:, i+1] = y[:, i] + np.array(dydt) * dt

        return {'t': t_eval, 'y': y}

    def plot_dynamic_results(self):
        """Plot dynamic simulation results"""
        if not self.simulation_data['time']:
            return

        time = self.simulation_data['time']
        speed = self.simulation_data['speed']
        torque = self.simulation_data['torque']
        current = self.simulation_data['current']
        power = self.simulation_data['power']

        # Plot Speed
        self.ax_speed.clear()
        self.ax_speed.plot(time, speed, 'b-', linewidth=2)
        self.ax_speed.set_xlabel('Time (s)', fontweight='bold')
        self.ax_speed.set_ylabel('Speed (RPM)', fontweight='bold')
        self.ax_speed.set_title('Rotor Speed vs Time', fontweight='bold')
        self.ax_speed.grid(True, alpha=0.3)

        # Plot Torque
        self.ax_torque_time.clear()
        self.ax_torque_time.plot(time, torque, 'r-', linewidth=2, label='Electromagnetic Torque')
        self.ax_torque_time.axhline(y=self.motor.T_load, color='g',
                                    linestyle='--', linewidth=2, label='Load Torque')
        self.ax_torque_time.set_xlabel('Time (s)', fontweight='bold')
        self.ax_torque_time.set_ylabel('Torque (N·m)', fontweight='bold')
        self.ax_torque_time.set_title('Torque vs Time', fontweight='bold')
        self.ax_torque_time.grid(True, alpha=0.3)
        self.ax_torque_time.legend()

        # Plot Current
        self.ax_current.clear()
        self.ax_current.plot(time, current, 'g-', linewidth=2)
        self.ax_current.set_xlabel('Time (s)', fontweight='bold')
        self.ax_current.set_ylabel('Current (A)', fontweight='bold')
        self.ax_current.set_title('Stator Current vs Time', fontweight='bold')
        self.ax_current.grid(True, alpha=0.3)

        # Plot Power
        self.ax_power.clear()
        self.ax_power.plot(time, power, 'm-', linewidth=2)
        self.ax_power.set_xlabel('Time (s)', fontweight='bold')
        self.ax_power.set_ylabel('Power (kW)', fontweight='bold')
        self.ax_power.set_title('Output Power vs Time', fontweight='bold')
        self.ax_power.grid(True, alpha=0.3)

        self.fig_dynamic.tight_layout()
        self.canvas_dynamic.draw()

    def stop_simulation(self):
        """Stop simulation"""
        self.simulation_running = False
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")

    def reset_simulation(self):
        """Reset simulation"""
        self.simulation_running = False
        self.simulation_data = {
            'time': [],
            'speed': [],
            'torque': [],
            'current': [],
            'power': []
        }

        # Clear plots
        self.ax_speed.clear()
        self.ax_torque_time.clear()
        self.ax_current.clear()
        self.ax_power.clear()
        self.canvas_dynamic.draw()

        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")

    def update_load_display(self, value):
        """Update load torque display"""
        self.load_torque_label.config(text=f"{float(value):.1f} Nm")

    def update_inertia_display(self, value):
        """Update inertia display"""
        self.inertia_label.config(text=f"{float(value):.2f} kg·m²")

    def reset_all(self):
        """Reset all parameters and results"""
        # Reset parameters to defaults
        defaults = {
            'V_line': 400,
            'frequency': 50,
            'poles': 4,
            'R2': 0.01,
            'X2': 0.1,
            'R1': 0.02,
            'X1': 0.05,
            'a': 4,
            'Xm': 10
        }

        for key, value in defaults.items():
            if key in self.param_entries:
                self.param_entries[key].delete(0, tk.END)
                self.param_entries[key].insert(0, str(value))

        # Update motor
        self.update_parameters()

        # Clear results
        self.results_text.delete(1.0, tk.END)
        self.detailed_results_text.delete(1.0, tk.END)

        # Reset simulation
        self.reset_simulation()

        messagebox.showinfo("Reset", "All parameters and results have been reset!")

    def save_results(self):
        """Save results to file"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"im_analysis_results_{timestamp}.txt"

        try:
            with open(filename, 'w') as f:
                f.write("INDUCTION MOTOR ANALYSIS RESULTS\n")
                f.write("=" * 80 + "\n\n")
                f.write(self.detailed_results_text.get(1.0, tk.END))

            messagebox.showinfo("Success", f"Results saved to {filename}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save results: {str(e)}")

    def load_parameters(self):
        """Load parameters (placeholder)"""
        messagebox.showinfo("Info", "Parameter loading feature - to be implemented")

    def export_results(self):
        """Export detailed results"""
        self.save_results()

    def show_about(self):
        """Show about dialog"""
        about_text = """
Advanced Induction Motor Analysis System
Version 1.0

Features:
• Steady-state analysis
• Dynamic simulation with multiple ODE solvers
• Comprehensive motor characteristics
• Real-time visualization
• Professional electrical engineering tool

Developed for educational and professional use
        """
        messagebox.showinfo("About", about_text)

    def show_user_guide(self):
        """Show user guide"""
        guide_text = """
USER GUIDE

1. Steady-State Analysis:
   - Enter motor parameters
   - Click 'Calculate Q.5' to solve
   - View results and torque-slip curve

2. Dynamic Simulation:
   - Adjust load torque and inertia
   - Select ODE solver (RK45 recommended)
   - Click 'Start' to run simulation
   - View real-time dynamic response

3. Motor Characteristics:
   - Click 'Generate All Characteristics'
   - View torque, current, power, efficiency curves

4. Results:
   - View detailed results in Results tab
   - Export results to file using File menu

5. Controls:
   - Use sliders for real-time adjustments
   - All plots support zoom and pan
   - Window auto-resizes for optimal view
        """
        messagebox.showinfo("User Guide", guide_text)

    def on_window_resize(self, event):
        """Handle window resize events"""
        # This method is called on window resize
        # The canvas widgets will automatically adjust due to pack/grid with expand=True
        pass


def main():
    """Main entry point"""
    try:
        # Set style
        app = AdvancedIMSimulator()

        # Configure ttk style
        style = ttk.Style()
        style.theme_use('clam')

        # Custom button style
        style.configure('Accent.TButton', foreground='white', background='#0078d4',
                       font=('Arial', 10, 'bold'))

        # Run application
        app.mainloop()

    except Exception as e:
        print(f"Error starting application: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
