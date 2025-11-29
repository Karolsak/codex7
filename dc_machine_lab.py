"""
DC Machine Laboratory with GUI for steady-state calculations and dynamic simulation.
The tool solves the posed lap-wound armature problem and provides interactive controls
for exploring DC motor behavior with Euler and RK45 solvers.
"""

import math
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp


class DCMachineModel:
    """Mathematical model and utility calculations for a DC motor."""

    def __init__(self):
        # Default parameters based on the prompt
        self.poles = 6
        self.conductors = 720
        self.flux_wb = 0.02035  # 20.35 mWb
        self.armature_current = 78.0
        self.induced_voltage = 420.0
        self.armature_resistance = 0.25
        self.armature_inductance = 0.015
        self.inertia = 0.35
        self.friction = 0.002
        self.supply_voltage = 440.0
        self.winding_type = "lap"
        self.time_constant = 0.01

        # Dynamic state variables
        self.state = np.array([0.0, 0.0])  # [Ia (A), omega (rad/s)]

    @property
    def parallel_paths(self):
        if self.winding_type.lower() == "lap":
            return max(1, int(self.poles))
        return 2  # Wave wound default

    @property
    def torque_constant(self):
        return (self.poles * self.conductors) / (2 * math.pi * self.parallel_paths)

    @property
    def back_emf_constant(self):
        # Same constant as torque constant for SI units in this formulation
        return (self.poles * self.conductors) / (2 * math.pi * self.parallel_paths)

    def compute_steady_state(self):
        """
        Compute electromagnetic torque and speed from problem statement values.
        Returns torque (N·m) and speed (rpm).
        """
        torque = (
            self.poles
            * self.conductors
            * self.flux_wb
            * self.armature_current
            / (2 * math.pi * self.parallel_paths)
        )
        speed_rpm = (
            60
            * self.induced_voltage
            * self.parallel_paths
            / (self.poles * self.flux_wb * self.conductors)
        )
        return torque, speed_rpm

    def derivatives(self, _t, state, flux_scale=1.0, load_torque=0.0):
        ia, omega = state
        flux = max(1e-5, self.flux_wb * flux_scale)
        emf = self.back_emf_constant * flux * omega
        dia_dt = (self.supply_voltage - emf - ia * self.armature_resistance) / self.armature_inductance
        torque = self.torque_constant * flux * ia
        domega_dt = (torque - load_torque - self.friction * omega) / self.inertia
        return np.array([dia_dt, domega_dt])

    def step_euler(self, dt, flux_scale, load_torque):
        derivs = self.derivatives(0, self.state, flux_scale, load_torque)
        self.state = self.state + dt * derivs
        return self.state

    def step_rk45(self, dt, flux_scale, load_torque):
        sol = solve_ivp(
            lambda t, y: self.derivatives(t, y, flux_scale, load_torque),
            [0, dt],
            self.state,
            method="RK45",
            max_step=dt,
        )
        self.state = sol.y[:, -1]
        return self.state

    def reset_state(self):
        self.state = np.array([0.0, 0.0])


class DCMachineLab(tk.Tk):
    """Tkinter GUI that wraps the DC machine model and visualization."""

    def __init__(self):
        super().__init__()
        self.title("DC Machine Laboratory")
        self.geometry("1350x900")
        self.minsize(1100, 750)

        self.model = DCMachineModel()
        self.running = False
        self.time_history = []
        self.ia_history = []
        self.omega_history = []
        self.torque_history = []

        self._build_menu()
        self._build_layout()
        self._configure_responsiveness()
        self._update_steady_state_labels()

    def _build_menu(self):
        menubar = tk.Menu(self)
        file_menu = tk.Menu(menubar, tearoff=0)
        file_menu.add_command(label="Reset", command=self.reset_all)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.destroy)
        menubar.add_cascade(label="File", menu=file_menu)

        sim_menu = tk.Menu(menubar, tearoff=0)
        sim_menu.add_command(label="Start", command=self.start_simulation)
        sim_menu.add_command(label="Stop", command=self.stop_simulation)
        menubar.add_cascade(label="Simulation", menu=sim_menu)

        help_menu = tk.Menu(menubar, tearoff=0)
        help_menu.add_command(label="About", command=self._show_about)
        menubar.add_cascade(label="Help", menu=help_menu)
        self.config(menu=menubar)

    def _build_layout(self):
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True)

        self.tab_input = ttk.Frame(self.notebook)
        self.tab_dynamic = ttk.Frame(self.notebook)
        self.tab_advanced = ttk.Frame(self.notebook)

        self.notebook.add(self.tab_input, text="Input & Steady-State")
        self.notebook.add(self.tab_dynamic, text="Dynamic Simulation")
        self.notebook.add(self.tab_advanced, text="Advanced Analysis")

        self._build_input_tab()
        self._build_dynamic_tab()
        self._build_advanced_tab()

    def _configure_responsiveness(self):
        for tab in (self.tab_input, self.tab_dynamic, self.tab_advanced):
            tab.grid_rowconfigure(0, weight=1)
            tab.grid_columnconfigure(1, weight=1)
        self.bind("<Configure>", self._on_resize)

    def _build_input_tab(self):
        left = ttk.LabelFrame(self.tab_input, text="Input Parameters", padding=10)
        left.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)

        entries = {
            "poles": ("Poles", self.model.poles),
            "conductors": ("Conductors", self.model.conductors),
            "flux_wb": ("Flux per pole (Wb)", self.model.flux_wb),
            "armature_current": ("Armature current Ia (A)", self.model.armature_current),
            "induced_voltage": ("Induced voltage E (V)", self.model.induced_voltage),
            "armature_resistance": ("Armature resistance Ra (Ω)", self.model.armature_resistance),
            "armature_inductance": ("Armature inductance La (H)", self.model.armature_inductance),
            "inertia": ("Rotor inertia J (kg·m²)", self.model.inertia),
            "friction": ("Friction B (N·m·s)", self.model.friction),
            "supply_voltage": ("Supply voltage V (V)", self.model.supply_voltage),
        }
        self.inputs = {}
        for row, (key, (label, default)) in enumerate(entries.items()):
            ttk.Label(left, text=label).grid(row=row, column=0, sticky="w", pady=4)
            ent = ttk.Entry(left, width=18)
            ent.insert(0, str(default))
            ent.grid(row=row, column=1, sticky="ew", pady=4)
            self.inputs[key] = ent

        self.winding_choice = tk.StringVar(value=self.model.winding_type)
        ttk.Label(left, text="Winding type").grid(row=len(entries), column=0, sticky="w", pady=4)
        winding_box = ttk.Combobox(left, textvariable=self.winding_choice, values=["lap", "wave"], width=15)
        winding_box.grid(row=len(entries), column=1, sticky="ew", pady=4)

        ttk.Button(left, text="Update", command=self._update_model_from_inputs).grid(
            row=len(entries) + 1, column=0, columnspan=2, pady=6
        )

        right = ttk.LabelFrame(self.tab_input, text="Steady-State Results", padding=10)
        right.grid(row=0, column=1, sticky="nsew", padx=8, pady=8)
        right.grid_columnconfigure(1, weight=1)

        self.torque_label = ttk.Label(right, text="Torque: -- N·m", font=("Segoe UI", 12, "bold"))
        self.speed_label = ttk.Label(right, text="Speed: -- rpm", font=("Segoe UI", 12, "bold"))
        self.paths_label = ttk.Label(right, text="Parallel paths: --")

        self.torque_label.grid(row=0, column=0, sticky="w", pady=5, columnspan=2)
        self.speed_label.grid(row=1, column=0, sticky="w", pady=5, columnspan=2)
        self.paths_label.grid(row=2, column=0, sticky="w", pady=5, columnspan=2)

        desc = (
            "Use the update button after modifying inputs."
            " The displayed torque and speed answer the given design problem"
            " for the selected parameters."
        )
        ttk.Label(right, text=desc, wraplength=360).grid(row=3, column=0, columnspan=2, sticky="w", pady=10)

    def _build_dynamic_tab(self):
        control = ttk.LabelFrame(self.tab_dynamic, text="Controls", padding=10)
        control.grid(row=0, column=0, sticky="nsw", padx=8, pady=8)

        self.flux_slider = tk.Scale(control, from_=0.5, to=1.5, resolution=0.01, orient="horizontal", label="Flux scale")
        self.flux_slider.set(1.0)
        self.flux_slider.pack(fill="x", pady=4)

        self.load_slider = tk.Scale(
            control,
            from_=0.0,
            to=300.0,
            resolution=1.0,
            orient="horizontal",
            label="Load torque (N·m)",
        )
        self.load_slider.set(50.0)
        self.load_slider.pack(fill="x", pady=4)

        ttk.Label(control, text="Solver").pack(anchor="w", pady=2)
        self.solver_choice = ttk.Combobox(control, values=["Euler", "RK45"], width=10)
        self.solver_choice.set("RK45")
        self.solver_choice.pack(fill="x", pady=2)

        ttk.Button(control, text="Start", command=self.start_simulation).pack(fill="x", pady=2)
        ttk.Button(control, text="Stop", command=self.stop_simulation).pack(fill="x", pady=2)
        ttk.Button(control, text="Reset", command=self.reset_all).pack(fill="x", pady=2)

        info = ttk.Label(control, text="Real-time simulation with selectable ODE solver.", wraplength=200)
        info.pack(fill="x", pady=8)

        viz = ttk.LabelFrame(self.tab_dynamic, text="Visualization", padding=10)
        viz.grid(row=0, column=1, sticky="nsew", padx=8, pady=8)
        viz.grid_rowconfigure(0, weight=1)
        viz.grid_columnconfigure(0, weight=1)

        self.figure = Figure(figsize=(7, 5), dpi=100)
        self.ax_speed = self.figure.add_subplot(311)
        self.ax_current = self.figure.add_subplot(312)
        self.ax_torque = self.figure.add_subplot(313)
        self.ax_speed.set_ylabel("Speed (rad/s)")
        self.ax_current.set_ylabel("Ia (A)")
        self.ax_torque.set_ylabel("Torque (N·m)")
        self.ax_torque.set_xlabel("Time (s)")

        self.canvas = FigureCanvasTkAgg(self.figure, master=viz)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        toolbar = NavigationToolbar2Tk(self.canvas, viz)
        toolbar.update()

    def _build_advanced_tab(self):
        frame = ttk.LabelFrame(self.tab_advanced, text="Comprehensive Analysis", padding=10)
        frame.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=8, pady=8)
        frame.grid_rowconfigure(1, weight=1)
        frame.grid_columnconfigure(0, weight=1)

        summary_btn = ttk.Button(frame, text="Generate Detailed Summary", command=self._update_advanced_summary)
        summary_btn.grid(row=0, column=0, sticky="w")

        self.summary_box = scrolledtext.ScrolledText(frame, height=20, wrap=tk.WORD)
        self.summary_box.grid(row=1, column=0, sticky="nsew", pady=6)

    def _on_resize(self, _event):
        # Autoscale plots when window changes
        self.figure.tight_layout()
        self.canvas.draw_idle()

    def _update_model_from_inputs(self):
        try:
            for key, entry in self.inputs.items():
                setattr(self.model, key, float(entry.get()))
            self.model.winding_type = self.winding_choice.get()
            self.model.reset_state()
            self.time_history.clear()
            self.ia_history.clear()
            self.omega_history.clear()
            self.torque_history.clear()
            self._update_steady_state_labels()
            messagebox.showinfo("Updated", "Parameters refreshed and state reset.")
        except ValueError:
            messagebox.showerror("Invalid input", "Please enter numeric values for all parameters.")

    def _update_steady_state_labels(self):
        torque, speed = self.model.compute_steady_state()
        self.torque_label.config(text=f"Torque: {torque:.2f} N·m")
        self.speed_label.config(text=f"Speed: {speed:.1f} rpm")
        self.paths_label.config(text=f"Parallel paths (A): {self.model.parallel_paths}")

    def start_simulation(self):
        self.running = True
        self._simulate_step(last_time=self.time_history[-1] if self.time_history else 0.0)

    def stop_simulation(self):
        self.running = False

    def reset_all(self):
        self.running = False
        self.model.reset_state()
        self.time_history.clear()
        self.ia_history.clear()
        self.omega_history.clear()
        self.torque_history.clear()
        self._clear_plots()
        self._update_steady_state_labels()

    def _clear_plots(self):
        for ax in (self.ax_speed, self.ax_current, self.ax_torque):
            ax.clear()
        self.ax_speed.set_ylabel("Speed (rad/s)")
        self.ax_current.set_ylabel("Ia (A)")
        self.ax_torque.set_ylabel("Torque (N·m)")
        self.ax_torque.set_xlabel("Time (s)")
        self.canvas.draw_idle()

    def _simulate_step(self, last_time):
        if not self.running:
            return

        dt = 0.02
        flux_scale = self.flux_slider.get()
        load_torque = self.load_slider.get()
        if self.solver_choice.get() == "Euler":
            state = self.model.step_euler(dt, flux_scale, load_torque)
        else:
            state = self.model.step_rk45(dt, flux_scale, load_torque)

        ia, omega = state
        torque = self.model.torque_constant * self.model.flux_wb * flux_scale * ia

        t = last_time + dt
        self.time_history.append(t)
        self.ia_history.append(ia)
        self.omega_history.append(omega)
        self.torque_history.append(torque)

        self._update_plots()
        self.after(int(dt * 1000), lambda: self._simulate_step(t))

    def _update_plots(self):
        self.ax_speed.clear()
        self.ax_current.clear()
        self.ax_torque.clear()

        self.ax_speed.plot(self.time_history, self.omega_history, color="tab:blue")
        self.ax_current.plot(self.time_history, self.ia_history, color="tab:orange")
        self.ax_torque.plot(self.time_history, self.torque_history, color="tab:green")

        self.ax_speed.set_ylabel("Speed (rad/s)")
        self.ax_current.set_ylabel("Ia (A)")
        self.ax_torque.set_ylabel("Torque (N·m)")
        self.ax_torque.set_xlabel("Time (s)")

        for ax in (self.ax_speed, self.ax_current, self.ax_torque):
            ax.grid(True, linestyle="--", alpha=0.5)

        self.figure.tight_layout()
        self.canvas.draw_idle()

    def _update_advanced_summary(self):
        torque, speed = self.model.compute_steady_state()
        ia0, omega0 = self.model.state
        summary = [
            "Comprehensive Analysis",
            "---------------------",
            f"Winding type: {self.model.winding_type.title()} ({self.model.parallel_paths} parallel paths)",
            f"Torque constant k_t: {self.model.torque_constant:.2f} N·m/A",
            f"Back-EMF constant k_e: {self.model.back_emf_constant:.2f} V·s/rad",
            f"Steady-state torque: {torque:.2f} N·m",
            f"Steady-state speed: {speed:.1f} rpm",
            f"Current state Ia: {ia0:.2f} A, ω: {omega0:.2f} rad/s",
            "Practical guidance:",
            " - Increase flux scale to emulate field strengthening.",
            " - Increase load torque to study sag and recovery with RK45 vs Euler.",
            " - Use the summary to configure test benches or teaching labs.",
        ]
        self.summary_box.delete("1.0", tk.END)
        self.summary_box.insert(tk.END, "\n".join(summary))

    def _show_about(self):
        messagebox.showinfo(
            "About",
            "DC Machine Laboratory\n"
            "Interactive tool with steady-state answers, real-time ODE solvers,\n"
            "and autoscaling plots for electrical engineering experiments.",
        )


def main():
    app = DCMachineLab()
    app.mainloop()


if __name__ == "__main__":
    main()
