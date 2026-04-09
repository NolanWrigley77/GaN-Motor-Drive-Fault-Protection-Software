import customtkinter
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import random
import threading
import queue
import time
import serial
import test_script_MotorDrive

customtkinter.set_appearance_mode("dark")
customtkinter.set_default_color_theme("blue")


class InfineonGUI(customtkinter.CTk):
    def __init__(self):
        super().__init__()
        self.title("Infineon Ref Board GUI")
        self.geometry("800x500")

        # ---------------- UART state ----------------
        self.ser = None
        self.running = True
        self.rx_queue = queue.Queue()
        self.serial_thread = None
        self.t0 = None
        self.ia = None
        self.ib = None
        self.ic = None
        self.vmes_mv = None
        self.status = "--"
        self.run_state = "--"
        # ---------------- Plot Variables ----------------
        self.last_plot_time = 0.0
        self.PLOT_INTERVAL = 2.0      # seconds
        self.plot_initialized = True

        # Lock Declaration for polling
        self.pause_serial_reader = False


        # ---------------- Navigation Bar ----------------
        self.navbar = customtkinter.CTkFrame(self, corner_radius=0)
        self.navbar.pack(side="top", fill="x")

        self.container = customtkinter.CTkFrame(self)
        self.container.pack(fill="both", expand=True)
        self.container.grid_rowconfigure(0, weight=1)
        self.container.grid_columnconfigure(0, weight=1)

        self.page_names = ["Main", "Motor Control", "Sensor", "History"]
        self.pages = {name: self._create_page(name) for name in self.page_names}

        self.nav_buttons = {}
        for name in self.page_names:
            btn = customtkinter.CTkLabel(
                self.navbar, text=name, text_color="white",
                fg_color="black", corner_radius=5,
                padx=20, pady=10, cursor="hand2"
            )
            btn.pack(side="left", padx=1, pady=1)
            btn.bind("<Button-1>", lambda e, n=name: self.show_frame(n))
            self.nav_buttons[name] = btn

        # Build pages
        self._build_main()
        self._build_motor()
        self._build_sensor()
        self._build_history()

        self.show_frame("Main")

        # Poll serial queue
        self.after(100, self._poll_serial_queue)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # -------------------------------------------------
    def _create_page(self, name):
        frame = customtkinter.CTkFrame(self.container)
        frame.grid(row=0, column=0, sticky="nsew")
        return frame

    def show_frame(self, name):
        self.pages[name].tkraise()
        for k, btn in self.nav_buttons.items():
            btn.configure(fg_color="blue" if k == name else "black")

    # -------------------------------------------------
    # MAIN PAGE
    # -------------------------------------------------
    def _build_main(self):
        page = self.pages["Main"]

        customtkinter.CTkLabel(
            page,
            text="GUI for the advanced GaN based motor drive for data center cooling applications",
            font=("Arial", 18)
        ).pack(pady=20)

        comm = customtkinter.CTkFrame(page)
        comm.pack(pady=10, padx=20, fill="x")

        customtkinter.CTkLabel(comm, text="Communication Setup",
                               font=("Arial", 18)).pack(pady=10)

        customtkinter.CTkLabel(comm, text="UART Port").pack()
        self.uart_port_entry = customtkinter.CTkEntry(comm)
        self.uart_port_entry.insert(0, "COM5")
        self.uart_port_entry.pack(pady=4)

        customtkinter.CTkLabel(comm, text="Baudrate").pack()
        self.uart_baud_entry = customtkinter.CTkEntry(comm)
        self.uart_baud_entry.insert(0, "115200")
        self.uart_baud_entry.pack(pady=4)

        customtkinter.CTkLabel(comm, text="Parity").pack()
        self.uart_parity_select = customtkinter.CTkOptionMenu(
            comm, values=["None", "Even", "Odd"]
        )
        self.uart_parity_select.pack(pady=4)

        btns = customtkinter.CTkFrame(comm)
        btns.pack(pady=8)

        customtkinter.CTkButton(
            btns, text="Connect UART", command=self.connect_uart
        ).pack(side="left", padx=6)

        customtkinter.CTkButton(
            btns, text="Disconnect", command=self.disconnect_uart
        ).pack(side="left", padx=6)

        self.uart_status = customtkinter.CTkLabel(
            comm, text="Status: Disconnected"
        )
        self.uart_status.pack(pady=4)

    # -------------------------------------------------
    # MOTOR PAGE
    # -------------------------------------------------
    def _build_motor(self):
        page = self.pages["Motor Control"]

        customtkinter.CTkLabel(
            page, text="Motor Control", font=("Arial", 18)
        ).pack(pady=20)
        # ---------------- Motor control method ----------------
        self.control_methods = ["Sensorless Current FOC","FOC Current", "Sensorless Torque FOC","Sensorless Speed FOC", "Speed FOC"]
        self.control_method_var = customtkinter.StringVar(value=self.control_methods[0])
        ctrl_row = customtkinter.CTkFrame(page,fg_color="transparent")
        ctrl_row.pack(pady=(0, 10))

        customtkinter.CTkLabel(ctrl_row, text="Control Method:").pack(side="left", padx=(0, 10))

        self.control_method_menu = customtkinter.CTkOptionMenu(ctrl_row, values=self.control_methods, variable=self.control_method_var, command=self.on_control_method_change, width=200)
        self.control_method_menu.pack(side="left")

        customtkinter.CTkLabel(page, text="Speed").pack()
        self.speed_slider = customtkinter.CTkSlider(page, from_=0, to=100, width=300)
        self.speed_slider.pack(pady=10)

        self.motor_status = customtkinter.CTkLabel(page, text="Motor stopped")
        self.motor_status.pack(pady=10)

        customtkinter.CTkButton(page, text="Start Motor", command=self.start_motor).pack(pady=5)
        customtkinter.CTkButton(page, text="Stop Motor", command=self.stop_motor).pack(pady=5)

    def start_motor(self):
        self.motor_status.configure(text=f"Motor running at {int(self.speed_slider.get())}")

    def stop_motor(self):
        self.motor_status.configure(text="Motor stopped")

    def on_control_method_change(self, choice: str):
        # optional: show selection somewhere on the motor page
        if hasattr(self, "motor_status"):
            self.motor_status.configure(text=f"Selected control: {choice}")

    # -------------------------------------------------
    # SENSOR PAGE
    # -------------------------------------------------
    def _build_sensor(self):
        page = self.pages["Sensor"]
        
        # Status indicators at top
        status_container = customtkinter.CTkFrame(page)
        status_container.pack(pady=10, padx=20, fill="x")
        
        indicators_row = customtkinter.CTkFrame(status_container)
        indicators_row.pack(pady=5)
        
        customtkinter.CTkLabel(
            indicators_row, text="STATUS:", font=("Arial", 12, "bold")
        ).pack(side="left", padx=5)
        
        self.status_indicator = customtkinter.CTkLabel(
            indicators_row, text="--", font=("Arial", 12, "bold"),
            fg_color="gray30", corner_radius=5, padx=12, pady=5, width=80
        )
        self.status_indicator.pack(side="left", padx=5)
        
        customtkinter.CTkLabel(
            indicators_row, text="RUN:", font=("Arial", 12, "bold")
        ).pack(side="left", padx=5)
        
        self.run_indicator = customtkinter.CTkLabel(
            indicators_row, text="--", font=("Arial", 12, "bold"),
            fg_color="gray30", corner_radius=5, padx=12, pady=5, width=80
        )
        self.run_indicator.pack(side="left", padx=5)

        self.sensor_label = customtkinter.CTkLabel(
            page,
            text="Temperature: -- °C\nVoltage S1: -- V\nVoltage S2: -- V\nVoltage S3: -- V\nVoltage S4: -- V",
            font=("Arial", 14)
        )
        self.sensor_label.pack(pady=10)

        plot_frame = customtkinter.CTkFrame(page)
        plot_frame.pack(fill="both", expand=True, padx=20, pady=20)

        # Create two side-by-side frames for plots
        left_plot_frame = customtkinter.CTkFrame(plot_frame, fg_color="transparent")
        left_plot_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))
        
        right_plot_frame = customtkinter.CTkFrame(plot_frame, fg_color="transparent")
        right_plot_frame.pack(side="right", fill="both", expand=True, padx=(5, 0))

        # --- Temperature plot (left)
        self.fig_temp, self.ax_temp = plt.subplots(figsize=(4, 3))
        self.ax_temp.set_title("Temperature")
        self.ax_temp.set_xlabel("Time (s)")
        self.ax_temp.set_ylabel("Temperature (°C)")

        # Temperature thresholds (from main.c)
        TEMP_WARNING_C = 31
        TEMP_HARD_FAULT_C = 33

        # Add threshold lines
        self.ax_temp.axhline(y=TEMP_WARNING_C, color="yellow", linestyle="--", linewidth=2.5, label="Warning Threshold")
        self.ax_temp.axhline(y=TEMP_HARD_FAULT_C, color="red", linestyle="--", linewidth=2.5, label="Hard Fault Threshold")

        # Temperature ticks: major every 10°C, minor every 2.5°C
        self.ax_temp.yaxis.set_major_locator(MultipleLocator(10))
        self.ax_temp.yaxis.set_minor_locator(MultipleLocator(2.5))
        self.ax_temp.yaxis.set_major_formatter(FormatStrFormatter("%.0f"))

        self.ax_temp.grid(True, which="major", linewidth=1.0)
        self.ax_temp.grid(True, which="minor", linewidth=0.4, alpha=0.4)

        # Set initial reasonable limits
        self.ax_temp.set_ylim(10, 50)

        # Temperature line
        (self.temp_line,) = self.ax_temp.plot([], [], color="black", label="Temp (°C)", linewidth=2)
        self.ax_temp.legend(loc="upper left")
        self.fig_temp.tight_layout()

        self.canvas_temp = FigureCanvasTkAgg(self.fig_temp, master=left_plot_frame)
        self.canvas_temp.get_tk_widget().pack(fill="both", expand=True)

        # --- Current plot (right)
        self.fig_curr, self.ax_curr = plt.subplots(figsize=(4, 3))
        self.ax_curr.set_title("Phase Currents")
        self.ax_curr.set_xlabel("Time (s)")
        self.ax_curr.set_ylabel("Phase Current (A)")

        self.ax_curr.grid(True, which="major", linewidth=1.0)
        self.ax_curr.grid(True, which="minor", linewidth=0.4, alpha=0.4)

        # Current lines
        (self.ia_line,)   = self.ax_curr.plot([], [], color="black", label="Ia (A)", linewidth=2)
        (self.ib_line,)   = self.ax_curr.plot([], [], color="red", label="Ib (A)", linewidth=2)
        (self.ic_line,)   = self.ax_curr.plot([], [], color="blue", label="Ic (A)", linewidth=2)

        self.ax_curr.legend(loc="upper left")
        self.fig_curr.tight_layout()

        self.canvas_curr = FigureCanvasTkAgg(self.fig_curr, master=right_plot_frame)
        self.canvas_curr.get_tk_widget().pack(fill="both", expand=True)

        # Data history
        self.time_data = []
        self.temp_data = []
        self.ia_data = []
        self.ib_data = []
        self.ic_data = []

    # -------------------------------------------------
    # HISTORY PAGE
    # -------------------------------------------------
    def _history_add_row(self, event: str, result: str):
        ts = time.strftime("%H:%M:%S")
        self.history_table.insert("", "end", values=(ts, event, result))

    # -------------------------------------------------
    # Run Selected Test Script: Selects active test to run
    # -------------------------------------------------

    def run_selected_test_script(self):
        script = self.test_script_select.get().strip()

        self.test_status_label.configure(text=f"Status: Running {script}...")
        self._history_add_row("TEST_START", script)

        def worker():
            try:
                if not self.ser or not self.ser.is_open:
                    self.after(0, lambda: self._history_add_row("TEST_FAIL", "UART not connected"))
                    self.after(0, lambda: self.test_status_label.configure(text="Status: FAIL (not connected)"))
                    return

                # Stop Polling
                self.pause_serial_reader = True
                time.sleep(0.1)
                self.ser.reset_input_buffer()

                if script == "test_script_MotorDrive":
                    if hasattr(test_script_MotorDrive, "run_comm_test"):
                        ok, msg = test_script_MotorDrive.run_comm_test(
                            self.ser,
                            timeout_s=2.0
                        )
                    else:
                        ok, msg = False, "Test script missing run_comm_test()"
                else:
                    ok, msg = False, f"Unknown script: {script}"

                if ok:
                    self.after(0, lambda: self._history_add_row("TEST_PASS", msg))
                    self.after(0, lambda: self.test_status_label.configure(text=f"Status: PASS ({script})"))
                else:
                    self.after(0, lambda: self._history_add_row("TEST_FAIL", msg))
                    self.after(0, lambda: self.test_status_label.configure(text=f"Status: FAIL ({script})"))

            except Exception as e:
                self.after(0, lambda: self._history_add_row("TEST_ERROR", str(e)))
                self.after(0, lambda: self.test_status_label.configure(text=f"Status: ERROR ({script})"))
            finally:
                # Resume Polling
                self.pause_serial_reader = False

        threading.Thread(target=worker, daemon=True).start()

    # -------------------------------------------------

    def _build_history(self):
        page = self.pages["History"]
        columns = ("Time", "Event", "Result")
        self.history_table = ttk.Treeview(page, columns=columns, show="headings")
        for col in columns:
            self.history_table.heading(col, text=col)
        self.history_table.pack(padx=20, pady=20, fill="x")

        runner = customtkinter.CTkFrame(page)
        runner.pack(padx=20, pady=(0, 20), fill="x")

        customtkinter.CTkLabel(
            runner, text="Run Test Script", font=("Arial", 16)
        ).pack(pady=(10, 6))

        self.test_script_select = customtkinter.CTkOptionMenu(
            runner,
            values=["test_script_MotorDrive"],
        )
        self.test_script_select.pack(pady=6)
        self.test_script_select.set("test_script_MotorDrive")

        btn_row = customtkinter.CTkFrame(runner)
        btn_row.pack(pady=(6, 10))

        customtkinter.CTkButton(
            btn_row, text="Run Selected Test", command=self.run_selected_test_script
        ).pack(side="left", padx=6)

        self.test_status_label = customtkinter.CTkLabel(
            runner, text="Status: Idle"
        )
        self.test_status_label.pack(pady=(0, 10))

    # -------------------------------------------------
    # UART METHODS
    # -------------------------------------------------
    def connect_uart(self):
        try:
            port = self.uart_port_entry.get().strip()
            baud = int(self.uart_baud_entry.get().strip())
            parity = self.uart_parity_select.get()

            parity_map = {
                "None": serial.PARITY_NONE,
                "Even": serial.PARITY_EVEN,
                "Odd": serial.PARITY_ODD
            }

            self.ser = serial.Serial(
                port,
                baudrate=baud,
                parity=parity_map.get(parity, serial.PARITY_NONE),
                timeout=1
            )
            self.ser.reset_input_buffer()
            # also clear queued GUI messages so old TEMP/CUR doesn't update mid-test
            while not self.rx_queue.empty():
                try:
                    self.rx_queue.get_nowait()
                except queue.Empty:
                    break

            self.uart_status.configure(text=f"Status: Connected ({port})")

            if self.serial_thread is None or not self.serial_thread.is_alive():
                self.serial_thread = threading.Thread(
                    target=self._serial_reader, daemon=True
                )
                self.serial_thread.start()

        except Exception as e:
            self.uart_status.configure(text=f"Status: Error ({e})")

    def disconnect_uart(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.uart_status.configure(text="Status: Disconnected")

    def _serial_reader(self):
        """Background thread: reads UART lines and pushes parsed messages into rx_queue."""
        while self.running:
            # Pause Polling
            if self.pause_serial_reader:
                time.sleep(0.05)
                continue

            if not self.ser:
                time.sleep(0.2)
                continue

            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue

                print("RX:", line)
                
                # Try new format first: TEL STATUS=OK RUN=STOP TEMP=27.4C I3=0.45A I4=1.50A I5=1.91A VMES=3009
                if "TEL" in line and "TEMP=" in line:
                    try:
                        status = "--"
                        run = "--"
                        
                        # Parse STATUS if present
                        if "STATUS=" in line:
                            status = line.split("STATUS=")[1].split()[0]
                        
                        # Parse RUN if present
                        if "RUN=" in line:
                            run = line.split("RUN=")[1].split()[0]
                        
                        tC   = float(line.split("TEMP=")[1].split("C")[0])
                        i3   = float(line.split("I3=")[1].split("A")[0])
                        i4   = float(line.split("I4=")[1].split("A")[0])
                        i5   = float(line.split("I5=")[1].split("A")[0])
                        vmes = int(line.split("VMES=")[1].split()[0])

                        self.rx_queue.put(("TEL", status, run, tC, i3, i4, i5, vmes))
                        continue

                    except Exception as e:
                        print("TEL parse error (new format):", e)

                # Try older format: TEL: TEMP=..;CH3=...;CH4=...;CH5=...;VMES=...
                if line.startswith("TEL:"):
                    line = line[4:].lstrip()

                if ("TEMP=" in line and "CH3=" in line and "CH4=" in line and
                        "CH5=" in line and "VMES=" in line):

                    try:
                        # Julian-style parsing: split around fixed tokens
                        tC = float(line.split("TEMP=")[1].split(";")[0])
                        i3 = float(line.split("CH3=")[1].split(";")[0])
                        i4 = float(line.split("CH4=")[1].split(";")[0])
                        i5 = float(line.split("CH5=")[1].split(";")[0])

                        # VMES is last in your string; split(";")[0] still safe if it's last
                        vmes_mv = int(float(line.split("VMES=")[1].split(";")[0]))

                        self.rx_queue.put(("TEMP", 0, 0, tC))
                        self.rx_queue.put(("CUR", i3, i4, i5))
                        self.rx_queue.put(("VMES", 0, vmes_mv))

                    except Exception as e:
                        print("Serial/parse error (older format):", e, "| line=", line)

                    continue

                # Everything else ignored here (OK/ERR/etc.)

            except Exception as e:
                print("Serial reader error:", e)
                time.sleep(0.1)

    def _poll_serial_queue(self):
        if not self.plot_initialized:
            self.t0 = None
            self.last_plot_time = 0.0
            self.time_data.clear()
            self.temp_data.clear()
            self.ia_data.clear()
            self.ib_data.clear()
            self.ic_data.clear()
            self.plot_initialized = True
        
        latest_tel = None
        latest_temp = None
        latest_curr = None
        latest_vmes = None
        
        try:
            while True:
                msg = self.rx_queue.get_nowait()
                if msg[0] == "TEL":
                    latest_tel = msg
                elif msg[0] == "TEMP":
                    latest_temp = msg
                elif msg[0] == "CUR":
                    latest_curr = msg
                elif msg[0] == "VMES":
                    latest_vmes = msg
        except queue.Empty:
            pass

        # Handle new format (TEL)
        if latest_tel:
            _, self.status, self.run_state, tC_raw, self.ia, self.ib, self.ic, self.vmes_mv = latest_tel
            now = time.time()
            
            # Update status indicators with color coding
            status_colors = {
                "OK": "green",
                "WARN": "orange",
                "FAULT": "red",
                "--": "gray30"
            }
            run_colors = {
                "STOP": "gray40",
                "RUN": "green",
                "FAULT": "red",
                "--": "gray30"
            }
            
            self.status_indicator.configure(
                text=self.status,
                fg_color=status_colors.get(self.status, "gray30")
            )
            self.run_indicator.configure(
                text=self.run_state,
                fg_color=run_colors.get(self.run_state, "gray30")
            )

            if self.t0 is None:
                self.t0 = now
                self.last_plot_time = now
                
            # Only update plot if PLOT_INTERVAL has passed
            if (now - self.last_plot_time) >= self.PLOT_INTERVAL:

                # Compute relative time for x-axis
                rel_time = now - self.t0

                # Append temperature for plotting
                self.temp_data.append(tC_raw)
                self.ia_data.append(self.ia if self.ia is not None else 0.0)
                self.ib_data.append(self.ib if self.ib is not None else 0.0)
                self.ic_data.append(self.ic if self.ic is not None else 0.0)
                self.time_data.append(rel_time)

                self.last_plot_time += self.PLOT_INTERVAL

                # Limit history
                MAX_POINTS = 250
                self.temp_data = self.temp_data[-MAX_POINTS:]
                self.ia_data = self.ia_data[-MAX_POINTS:]
                self.ib_data = self.ib_data[-MAX_POINTS:]
                self.ic_data = self.ic_data[-MAX_POINTS:]
                self.time_data = self.time_data[-MAX_POINTS:]

                # Update line data
                self.temp_line.set_data(self.time_data, self.temp_data)
                self.ia_line.set_data(self.time_data, self.ia_data)
                self.ib_line.set_data(self.time_data, self.ib_data)
                self.ic_line.set_data(self.time_data, self.ic_data)

                # Autoscale axes
                self.ax_temp.relim()
                self.ax_temp.autoscale_view(scalex=True, scaley=True)
                self.ax_curr.relim()
                self.ax_curr.autoscale_view()

                self.canvas_temp.draw_idle()
                self.canvas_curr.draw_idle()

            # Update sensor label
            ia_txt = "--" if self.ia is None else f"{self.ia:.2f}"
            ib_txt = "--" if self.ib is None else f"{self.ib:.2f}"
            ic_txt = "--" if self.ic is None else f"{self.ic:.2f}"
            vmes_txt = "--" if self.vmes_mv is None else f"{self.vmes_mv/1000:.3f}"

            self.sensor_label.configure(
                text=f"Temperature: {tC_raw:.1f} °C\n"
                    f"DC Link Vmes: {vmes_txt} V\n"
                    f"Phase A Current: {ia_txt} A\n"
                    f"Phase B Current: {ib_txt} A\n"
                    f"Phase C Current: {ic_txt} A\n"
            )

        # Handle older format (TEMP/CUR/VMES)
        elif latest_temp:
            _, adc, mv, tC = latest_temp

            if latest_vmes:
                _, adc_vmes, mv_vmes = latest_vmes
                self.vmes_mv = mv_vmes

            if latest_curr:
                _, self.ia, self.ib, self.ic = latest_curr

            ia_txt = "--" if self.ia is None else f"{self.ia:.2f}"
            ib_txt = "--" if self.ib is None else f"{self.ib:.2f}"
            ic_txt = "--" if self.ic is None else f"{self.ic:.2f}"
            vmes_txt = "--" if self.vmes_mv is None else f"{self.vmes_mv / 1000:.3f}"

            self.sensor_label.configure(
                text=f"Temperature: {tC:.1f} °C\n"
                     f"DC Link Vmes: {vmes_txt} V\n"
                     f"Phase A Current: {ia_txt} A\n"
                     f"Phase B Current: {ib_txt} A\n"
                     f"Phase C Current: {ic_txt} A\n"
            )

            now = time.time()
            if self.t0 is None:
                self.t0 = now

            self.time_data.append(now)
            t0 = self.time_data[0]
            time_plot = [t - t0 for t in self.time_data]

            # Append histories
            self.temp_data.append(tC)
            self.ia_data.append(self.ia if self.ia is not None else 0.0)
            self.ib_data.append(self.ib if self.ib is not None else 0.0)
            self.ic_data.append(self.ic if self.ic is not None else 0.0)

            # Update line data
            self.temp_line.set_data(time_plot, self.temp_data)
            self.ia_line.set_data(time_plot, self.ia_data)
            self.ib_line.set_data(time_plot, self.ib_data)
            self.ic_line.set_data(time_plot, self.ic_data)

            # Autoscale both axes
            self.ax_temp.relim()
            self.ax_temp.autoscale_view(scalex=True, scaley=True)
            self.ax_curr.relim()
            self.ax_curr.autoscale_view()

            self.canvas_temp.draw_idle()
            self.canvas_curr.draw_idle()

        self.after(100, self._poll_serial_queue)

    def on_close(self):
        self.running = False
        self.disconnect_uart()
        self.destroy()


if __name__ == "__main__":
    InfineonGUI().mainloop()
