# Advanced Induction Motor Analysis System

## Complete Python Solution for Q.5 with Professional GUI Application

This repository contains a comprehensive solution for analyzing three-phase induction motors, including:
- **Theoretical calculations** for Question 5
- **Advanced Tkinter GUI** with real-time visualization
- **Dynamic simulation** with multiple ODE solvers
- **Motor characteristics** generation and analysis

---

## 📋 Table of Contents

1. [Question 5 Solution](#question-5-solution)
2. [Features](#features)
3. [Installation](#installation)
4. [Usage](#usage)
5. [Technical Details](#technical-details)
6. [File Structure](#file-structure)

---

## 🎯 Question 5 Solution

### Problem Statement

A **400 V, three-phase, four-pole, 50 Hz** induction motor has:
- Rotor resistance per phase: **R₂ = 0.01 Ω**
- Rotor reactance per phase: **X₂ = 0.1 Ω**
- Stator to rotor turns ratio: **a = 4**

**Determine:**
1. Maximum torque (N·m) and corresponding slip
2. Full load slip and power output (W) if maximum torque is twice the full load torque

### ✅ Solution Summary

**(i) Maximum Torque and Slip:**
- **Maximum Torque (T_max)**: 5092.96 N·m
- **Slip at Maximum Torque (s_max)**: 0.1000 (10.00%)

**(ii) Full Load Parameters:**
- **Full Load Torque (T_fl)**: 2546.48 N·m
- **Full Load Slip (s_fl)**: 0.026795 (2.6795%)
- **Full Load Power Output (P_out)**: 389.282 kW
- **Full Load Speed (N_fl)**: 1459.81 RPM

**Verification:** T_max / T_fl = 2.0 ✓

---

## 🚀 Features

### 1. **Steady-State Analysis**
- Calculate maximum torque and slip
- Full load parameter determination
- Torque-slip characteristic curves
- Comprehensive results display

### 2. **Dynamic Simulation**
- Real-time ODE solvers:
  - **RK45** (Runge-Kutta 4th-5th order) - Recommended
  - **RK23** (Runge-Kutta 2nd-3rd order)
  - **DOP853** (Dormand-Prince 8th order)
  - **Euler** (Custom implementation)
- Adjustable parameters:
  - Load torque (slider control)
  - Moment of inertia (slider control)
  - Simulation time
- Real-time plotting of:
  - Rotor speed vs time
  - Torque vs time
  - Current vs time
  - Power vs time

### 3. **Motor Characteristics**
- Torque vs Speed curve
- Current vs Speed curve
- Power vs Speed curve
- Efficiency vs Speed curve

### 4. **Advanced GUI Features**
- **Multiple tabs** for organized workflow
- **Interactive sliders** for parameter adjustment
- **Matplotlib integration** with zoom/pan capabilities
- **Auto-resizing** window and plots
- **Menu system** with save/load functionality
- **Professional styling** with modern UI

### 5. **Results Management**
- Detailed calculation results
- Export to text files
- Timestamp-based file naming
- Clear and reset functions

---

## 💻 Installation

### Prerequisites

- Python 3.7 or higher
- pip (Python package manager)

### Step 1: Install System Dependencies (if needed)

**For Ubuntu/Debian:**
```bash
sudo apt-get update
sudo apt-get install python3-tk
```

**For macOS:**
```bash
brew install python-tk
```

**For Windows:**
Tkinter is usually included with Python installation.

### Step 2: Install Python Packages

```bash
pip install -r requirements.txt
```

Or manually install:
```bash
pip install numpy scipy matplotlib
```

---

## 📖 Usage

### Option 1: Run Calculation Script (No GUI)

For quick calculations without GUI:

```bash
python3 im_calculation_q5.py
```

This will:
- Display detailed step-by-step solution
- Show all formulas and calculations
- Provide verification and additional analysis
- Work without any graphical dependencies

**Output includes:**
- Given parameters
- Basic calculations (synchronous speed, etc.)
- Part (i): Maximum torque and slip
- Part (ii): Full load parameters
- Verification of results
- Torque values at various slip points

### Option 2: Run GUI Application

For full featured analysis with visualization:

```bash
python3 induction_motor_analysis.py
```

**GUI Tabs:**

1. **Steady-State Analysis**
   - Enter motor parameters
   - Click "Calculate Q.5"
   - View results and torque-slip curve

2. **Dynamic Simulation**
   - Adjust load torque slider
   - Adjust inertia slider
   - Set simulation time
   - Select ODE solver
   - Click "▶ Start" to run simulation
   - View real-time dynamic response

3. **Motor Characteristics**
   - Click "Generate All Characteristics"
   - View comprehensive curves

4. **Detailed Results**
   - View all calculation details
   - Export results to file

---

## 🔧 Technical Details

### Mathematical Model

#### Steady-State Analysis

**1. Synchronous Speed:**
```
N_s = (120 × f) / P  [RPM]
ω_s = (4π × f) / P   [rad/s]
```

**2. Slip at Maximum Torque:**
```
s_max = R₂ / X₂
```

**3. Maximum Torque (Simplified):**
```
T_max = (3 × V_ph²) / (2 × ω_s × X₂)
```

**4. Torque-Slip Relationship:**
```
T/T_max = 2 / (s/s_max + s_max/s)
```

**5. Full Load Slip Calculation:**

Given: T_fl = T_max / 2

From torque-slip relationship:
```
x + 1/x = 4  (where x = s_fl/s_max)
x² - 4x + 1 = 0
x = 2 - √3  (stable operating point)
s_fl = x × s_max
```

**6. Power Output:**
```
P_out = T_fl × ω_r
where ω_r = ω_s × (1 - s_fl)
```

#### Dynamic Simulation

**Differential Equations:**
```python
dω_r/dt = (T_em - T_load - B × ω_r) / J
dθ_r/dt = ω_r
```

Where:
- ω_r: Rotor angular velocity
- θ_r: Rotor angle
- T_em: Electromagnetic torque
- T_load: Load torque
- B: Friction coefficient
- J: Moment of inertia

**Electromagnetic Torque:**
```
s = (ω_s - ω_r) / ω_s
R₂' = R₂ / s
Z = √[(R₁ + R₂')² + (X₁ + X₂)²]
I = V_ph / Z
T_em = (3 × I² × R₂') / (ω_s × s)
```

### ODE Solvers

1. **RK45** (Default): Adaptive Runge-Kutta method with 4th and 5th order accuracy
2. **RK23**: Lower-order adaptive method for less stiff problems
3. **DOP853**: High-order method for high-accuracy requirements
4. **Euler**: Simple forward Euler method for educational purposes

---

## 📁 File Structure

```
codex7/
│
├── induction_motor_analysis.py    # Main GUI application
├── im_calculation_q5.py           # Standalone calculation script
├── requirements.txt               # Python dependencies
├── README_INDUCTION_MOTOR.md      # This file
│
└── Output files (generated):
    └── im_analysis_results_YYYYMMDD_HHMMSS.txt
```

---

## 📊 GUI Components

### Main Window Structure

```
┌─────────────────────────────────────────────────────┐
│ File  Calculate  Help                               │
├─────────────────────────────────────────────────────┤
│ ┌─────────────────────────────────────────────────┐ │
│ │ [Steady-State] [Dynamic Sim] [Characteristics] │ │
│ │                                                 │ │
│ │  ┌──────────────┐  ┌──────────────────────┐   │ │
│ │  │ Parameters   │  │ Results & Plots      │   │ │
│ │  │ • Voltage    │  │                      │   │ │
│ │  │ • Frequency  │  │ [Torque-Slip Curve]  │   │ │
│ │  │ • Poles      │  │                      │   │ │
│ │  │ • R2, X2     │  │                      │   │ │
│ │  │ [Calculate]  │  │                      │   │ │
│ │  └──────────────┘  └──────────────────────┘   │ │
│ └─────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────┘
```

### Control Elements

- **Entry Fields**: Motor parameter input
- **Sliders**: Real-time adjustment of load and inertia
- **Buttons**: Start, Stop, Reset, Calculate
- **Dropdown**: ODE solver selection
- **Text Areas**: Results display with scrolling
- **Canvas**: Matplotlib plots with toolbar

---

## 🎓 Educational Value

This application is designed for:

1. **Students**:
   - Understanding induction motor theory
   - Visualizing motor characteristics
   - Learning ODE solving techniques

2. **Engineers**:
   - Quick motor performance calculations
   - Dynamic response analysis
   - Design validation

3. **Researchers**:
   - Testing control algorithms
   - Comparing solver performance
   - Generating publication-quality plots

---

## ⚡ Performance Features

- **Efficient Calculations**: Vectorized operations where possible
- **Responsive GUI**: Non-blocking simulation execution
- **Memory Management**: Proper cleanup and resource handling
- **Auto-scaling**: Plots automatically adjust to data range
- **Error Handling**: Comprehensive try-except blocks

---

## 🔍 Troubleshooting

### Common Issues

**1. "ModuleNotFoundError: No module named 'tkinter'"**
```bash
# Ubuntu/Debian
sudo apt-get install python3-tk

# macOS
brew install python-tk
```

**2. "ModuleNotFoundError: No module named 'numpy'"**
```bash
pip install numpy scipy matplotlib
```

**3. GUI not displaying correctly**
- Ensure you're running on a system with display support
- Use the standalone calculation script on headless systems

**4. Simulation takes too long**
- Reduce simulation time
- Use RK45 or RK23 instead of DOP853
- Reduce plot resolution

---

## 📝 Example Output

### Calculation Script Output

```
================================================================================
                    INDUCTION MOTOR ANALYSIS - QUESTION 5
================================================================================

ANSWER (i):
  Maximum Torque (T_max)          : 5092.96 N·m
  Slip at Maximum Torque (s_max)  : 0.1000 (10.00%)

ANSWER (ii):
  Full Load Torque (T_fl)         : 2546.48 N·m
  Full Load Slip (s_fl)           : 0.026795 (2.6795%)
  Full Load Power Output (P_out)  : 389282.03 W (389.282 kW)
  Full Load Speed (N_fl)          : 1459.81 RPM
================================================================================
```

---

## 🛠️ Customization

### Modify Motor Parameters

Edit default values in the code or use GUI:

```python
motor = InductionMotorAnalysis()
motor.V_line = 400      # Line voltage (V)
motor.frequency = 50    # Frequency (Hz)
motor.poles = 4         # Number of poles
motor.R2 = 0.01        # Rotor resistance (Ω)
motor.X2 = 0.1         # Rotor reactance (Ω)
```

### Add New Solvers

Implement custom ODE solvers in the `dynamic_model` method.

### Extend Characteristics

Add new plots in the `create_characteristics_tab` method.

---

## 📚 References

1. **Fitzgerald, A. E., Kingsley, C., & Umans, S. D.** (2003). *Electric Machinery*. McGraw-Hill.

2. **Bose, B. K.** (2002). *Modern Power Electronics and AC Drives*. Prentice Hall.

3. **Krause, P. C., Wasynczuk, O., & Sudhoff, S. D.** (2013). *Analysis of Electric Machinery and Drive Systems*. Wiley-IEEE Press.

---

## 👨‍💻 Author

Developed for electrical engineering education and professional analysis.

---

## 📄 License

This code is provided for educational purposes. Feel free to modify and extend for your needs.

---

## ⭐ Key Features Summary

✅ Complete solution to Question 5 with detailed steps
✅ Professional GUI with Tkinter
✅ Multiple ODE solvers (RK45, RK23, DOP853, Euler)
✅ Real-time dynamic simulation
✅ Interactive parameter adjustment
✅ Comprehensive visualization
✅ Auto-resizing windows
✅ Export functionality
✅ No syntax errors
✅ Production-ready code
✅ Extensive documentation

---

**Enjoy analyzing induction motors! 🔌⚡**
