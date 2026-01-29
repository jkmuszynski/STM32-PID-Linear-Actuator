import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import threading
import collections
import time
from typing import Optional

# --- CONFIGURATION ---
BAUDRATE = 115200
HISTORY_LEN = 200
SERIAL_READ_INTERVAL = 0.005
PLOT_UPDATE_INTERVAL = 100
ERROR_THRESHOLD = 2  # mm
MIN_ERROR_SCALE = 5  # mm

# --- APPLICATION CLASS ---
class STM32PIDController:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("STM32 PID - Autoskaling")
        self.root.geometry("1000x800")
        
        # Serial communication
        self.serial_port: Optional[serial.Serial] = None
        self.is_running = False
        self.pending_target = 0
        self.read_thread: Optional[threading.Thread] = None
        
        # Data buffers
        self.data_target = collections.deque([0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.data_actual = collections.deque([0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.data_error = collections.deque([0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.data_pwm = collections.deque([0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        
        self._setup_ui()
        self._setup_plots()
        
    def _setup_ui(self):
        """Initialize all UI components"""
        # Top frame - Connection controls
        top_frame = tk.Frame(self.root, pady=10, bg="#DDD")
        top_frame.pack(side=tk.TOP, fill=tk.X)
        
        self.port_combobox = ttk.Combobox(top_frame, values=self._get_ports(), width=10)
        self.port_combobox.pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = tk.Button(top_frame, text="Połącz", command=self.connect_serial)
        self.connect_btn.pack(side=tk.LEFT)
        
        self.disconnect_btn = tk.Button(top_frame, text="Rozłącz", 
                                       command=self.disconnect_serial, state="disabled")
        self.disconnect_btn.pack(side=tk.LEFT)
        
        self.status_label = tk.Label(top_frame, text="---", bg="#DDD", width=15)
        self.status_label.pack(side=tk.LEFT)
        
        # Readout frame
        readout = tk.Frame(top_frame, bg="#DDD")
        readout.pack(side=tk.RIGHT, padx=20)
        
        self.lbl_act_target = tk.Label(readout, text="Cel: 0", font=("Arial", 10))
        self.lbl_act_target.pack(side=tk.LEFT, padx=10)
        
        tk.Label(readout, text="UCHYB:").pack(side=tk.LEFT)
        self.lbl_error_val = tk.Label(readout, text="0", font=("Arial", 14, "bold"))
        self.lbl_error_val.pack(side=tk.LEFT)
        
        self.lbl_mode_val = tk.Label(readout, text="---", font=("Arial", 10, "bold"))
        self.lbl_mode_val.pack(side=tk.LEFT, padx=10)
        
        # Bottom frame - Controls
        btm = tk.Frame(self.root, pady=10)
        btm.pack(side=tk.BOTTOM, fill=tk.X)
        
        # PID controls
        pid_frame = tk.LabelFrame(btm, text="PID")
        pid_frame.pack(side=tk.LEFT, padx=10, fill=tk.Y)
        
        tk.Label(pid_frame, text="P:").pack(side=tk.LEFT)
        self.entry_kp = tk.Entry(pid_frame, width=5)
        self.entry_kp.insert(0, "1.7")
        self.entry_kp.pack(side=tk.LEFT)
        
        tk.Label(pid_frame, text="I:").pack(side=tk.LEFT)
        self.entry_ki = tk.Entry(pid_frame, width=5)
        self.entry_ki.insert(0, "0.0")
        self.entry_ki.pack(side=tk.LEFT)
        
        tk.Label(pid_frame, text="D:").pack(side=tk.LEFT)
        self.entry_kd = tk.Entry(pid_frame, width=5)
        self.entry_kd.insert(0, "0.0")
        self.entry_kd.pack(side=tk.LEFT)
        
        tk.Button(pid_frame, text="Send", command=self.send_pid, bg="orange").pack(side=tk.LEFT, padx=5)
        
        # Control frame
        control_frame = tk.LabelFrame(btm, text="Sterowanie")
        control_frame.pack(side=tk.LEFT, padx=10, fill=tk.BOTH, expand=True)
        
        tk.Button(control_frame, text="TRYB MANUAL (Gałka)", 
                 command=self.send_manual, bg="lightblue").pack(fill=tk.X)
        
        scale_frame = tk.Frame(control_frame)
        scale_frame.pack(fill=tk.X, pady=5)
        
        self.scale = tk.Scale(scale_frame, from_=0, to=255, orient=tk.HORIZONTAL, 
                             command=self._on_slider_move)
        self.scale.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        self.lbl_pending = tk.Label(control_frame, text="Przesuń suwak...", fg="gray")
        self.lbl_pending.pack()
        
        tk.Button(control_frame, text="ZATWIERDŹ CEL [ENTER]", 
                 command=self.send_confirm_target, bg="#90EE90", 
                 font=("Arial", 10, "bold")).pack(fill=tk.X, pady=2)
    
    def _setup_plots(self):
        """Initialize matplotlib plots"""
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
        self.fig.subplots_adjust(hspace=0.2)
        
        # Position plot
        self.line_target, = self.ax1.plot([], [], 'g--', label='Cel', linewidth=1.5)
        self.line_actual, = self.ax1.plot([], [], 'b', label='Pozycja', linewidth=1.5)
        self.ax1.set_ylabel("Poz [mm]")
        self.ax1.legend(loc="upper right")
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_ylim(-10, 265)
        self.ax1.set_xlim(0, HISTORY_LEN)
        
        # Error plot
        self.line_error, = self.ax2.plot([], [], 'orange', label='Uchyb', linewidth=1.5)
        self.ax2.set_ylabel("Err [mm]")
        self.ax2.grid(True, alpha=0.3)
        self.ax2.axhline(y=0, color='k', linestyle='-', alpha=0.3, linewidth=0.8)
        
        # PWM plot
        self.line_pwm, = self.ax3.plot([], [], 'r', label='PWM', linewidth=1.5)
        self.ax3.set_ylabel("PWM")
        self.ax3.grid(True, alpha=0.3)
        self.ax3.set_ylim(-1100, 1100)
        self.ax3.axhline(y=0, color='k', linestyle='-', alpha=0.3, linewidth=0.8)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Start animation
        self.ani = FuncAnimation(self.fig, self._update_plot, 
                                interval=PLOT_UPDATE_INTERVAL, 
                                blit=False, cache_frame_data=False)
    
    # --- SERIAL COMMUNICATION ---
    def _get_ports(self) -> list:
        """Get list of available serial ports"""
        return [port.device for port in serial.tools.list_ports.comports()]
    
    def connect_serial(self):
        """Connect to selected serial port"""
        port = self.port_combobox.get()
        if not port:
            self.status_label.config(text="Wybierz port", fg="red")
            return
            
        try:
            self.serial_port = serial.Serial(port, BAUDRATE, timeout=0.01)
            self.serial_port.reset_input_buffer()
            self.status_label.config(text=f"Połączono: {port}", fg="green")
            self.connect_btn.config(state="disabled")
            self.disconnect_btn.config(state="normal")
            
            self.is_running = True
            self.read_thread = threading.Thread(target=self._read_serial_data, daemon=True)
            self.read_thread.start()
        except Exception as e:
            self.status_label.config(text=f"Błąd: {str(e)[:20]}", fg="red")
    
    def disconnect_serial(self):
        """Disconnect from serial port"""
        self.is_running = False
        
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        
        self.status_label.config(text="Rozłączono", fg="black")
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
    
    def _read_serial_data(self):
        """Background thread for reading serial data"""
        while self.is_running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    raw_data = self.serial_port.read(self.serial_port.in_waiting)
                    decoded = raw_data.decode('utf-8', errors='ignore')
                    
                    # Process only the latest frame
                    lines = decoded.split('\n')
                    for line in reversed(lines):
                        line = line.strip()
                        if line.startswith("DAT") and self._parse_data_line(line):
                            break
                else:
                    time.sleep(SERIAL_READ_INTERVAL)
            except Exception as e:
                print(f"Serial read error: {e}")
                time.sleep(SERIAL_READ_INTERVAL)
    
    def _parse_data_line(self, line: str) -> bool:
        """Parse data line from serial"""
        try:
            parts = line.split(',')
            if len(parts) < 8:
                return False
            
            targ = int(parts[2])
            akt = int(parts[3])
            err = int(parts[4])
            pwm = int(parts[5])
            mode = int(parts[8]) if len(parts) > 8 else 0
            
            self.data_target.append(targ)
            self.data_actual.append(akt)
            self.data_error.append(err)
            self.data_pwm.append(pwm)
            
            self.root.after(0, lambda: self._update_labels(err, mode, targ))
            return True
        except (ValueError, IndexError):
            return False
    
    def _update_labels(self, err: int, mode: int, targ: int):
        """Update GUI labels with current values"""
        color = "green" if abs(err) < ERROR_THRESHOLD else "red"
        self.lbl_error_val.config(text=f"{err} mm", fg=color)
        self.lbl_mode_val.config(text="AUTO (PC)" if mode == 1 else "MANUAL", fg="blue")
        self.lbl_act_target.config(text=f"Cel: {targ} mm")
    
    # --- CONTROL COMMANDS ---
    def send_pid(self):
        """Send PID parameters to STM32"""
        if not (self.serial_port and self.serial_port.is_open):
            return
        
        try:
            p = float(self.entry_kp.get())
            i = float(self.entry_ki.get())
            d = float(self.entry_kd.get())
            cmd = f"PID:{p:.2f}:{i:.2f}:{d:.2f}\n"
            self.serial_port.write(cmd.encode())
            print(f"PID SENT: {cmd.strip()}")
        except ValueError:
            print("Invalid PID values")
    
    def _on_slider_move(self, val):
        """Handle slider movement"""
        self.pending_target = int(float(val))
        self.lbl_pending.config(text=f"Do wysłania: {self.pending_target} mm", fg="gray")
    
    def send_confirm_target(self):
        """Send target position to STM32"""
        if self.serial_port and self.serial_port.is_open:
            cmd = f"SET:{self.pending_target}\n"
            self.serial_port.write(cmd.encode())
            self.lbl_pending.config(text=f"WYSŁANO: {self.pending_target} mm", fg="green")
    
    def send_manual(self):
        """Switch to manual mode"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(b"MAN\n")
    
    # --- PLOT UPDATE ---
    def _update_plot(self, frame):
        """Update plot with current data and auto-scale error axis"""
        if not self.is_running:
            return ()
        
        # Update data
        x_data = range(len(self.data_target))
        self.line_target.set_data(x_data, self.data_target)
        self.line_actual.set_data(x_data, self.data_actual)
        self.line_error.set_data(x_data, self.data_error)
        self.line_pwm.set_data(x_data, self.data_pwm)
        
        # Auto-scale error plot
        if len(self.data_error) > 0:
            max_error = max(abs(min(self.data_error)), abs(max(self.data_error)))
            limit = max(MIN_ERROR_SCALE, max_error * 1.2)
            self.ax2.set_ylim(-limit, limit)
        
        return ()

# --- MAIN ---
if __name__ == "__main__":
    root = tk.Tk()
    app = STM32PIDController(root)
    root.mainloop()