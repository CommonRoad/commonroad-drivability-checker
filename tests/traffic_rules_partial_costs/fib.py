import numpy as np
import matplotlib.pyplot as plt

# Define the time range
time = np.linspace(0, 1, 100)  # Normalized time from 0 to 1
t_f_label = r"$t_f$"  # Final time label
t_0_label = r"$t_0$"

# Define cost functions
duration = np.where((time > 0.2) & (time < 0.7), 1, 0)  # Binary duration
severity = np.where((time > 0.3) & (time < 0.7), np.sin((time - 0.3) * np.pi / 0.4), 0)  # Severity
severity[severity < 0] = 0  # Ensure non-negative values
duration_severity = severity  # Area under the curve for duration-severity
binary = np.where(np.any(duration > 0), 1, 0) * np.ones_like(time)  # Step function for binary

# Set up the figure and axes
fig, axs = plt.subplots(2, 2, figsize=(12, 8))
fig.subplots_adjust(hspace=0.4, wspace=0.3)

# Duration-Based Cost
axs[0, 0].step(time, duration, where="mid", color="blue", linewidth=2, label="Signal")
axs[0, 0].fill_between(time, duration, color="r", alpha=0.8, step="mid", label="Cost")
axs[0, 0].set_title("Duration",weight = "bold",fontsize=15)
axs[0, 0].set_xlabel("Time", fontsize = 16)
axs[0, 0].set_ylabel("Robustness Indicator", fontsize = 16)
axs[0, 0].set_yticks([0, 1])
axs[0, 0].set_xticks([0, 1])
axs[0, 0].set_yticklabels([0, 1], weight = "bold",fontsize = 15)  # Generic labels
axs[0, 0].set_xticklabels([t_0_label, t_f_label], weight = "bold",fontsize = 15)   # Generic labels
axs[0, 0].legend()# Generic label
# axs[0, 0].grid(True, linestyle="--", alpha=0.6)

# Duration-Severity Cost
axs[0, 1].plot(time, duration_severity, color="blue",linewidth=2, label="Signal")
axs[0, 1].fill_between(time, duration_severity, color="r", alpha=0.7, label="Cost")
axs[0, 1].set_title("Duration-Severity", weight = "bold",fontsize=15)
axs[0, 1].set_xlabel("Time", fontsize = 16)
axs[0, 1].set_ylabel("Robustness", fontsize = 16)
axs[0, 1].set_xticks([0, 1])
axs[0, 1].set_yticklabels(['', ''], weight = "bold",fontsize = 15)  # Generic labels
axs[0, 1].set_xticklabels([t_0_label, t_f_label], weight = "bold",fontsize = 15)  # Generic labelsaxs[0, 1].set_yticks([])
axs[0, 1].legend()
# axs[0, 1].grid(True, linestyle="--", alpha=0.6)

# Severity-Based Cost
axs[1, 0].plot(time, severity, color="blue",linewidth=2,  label="Signal")
axs[1, 0].axhline(np.max(severity), color="r", label="Cost")
axs[1, 0].set_title("Severity", weight = "bold",fontsize=15)
axs[1, 0].set_xlabel("Time", fontsize = 16)
axs[1, 0].set_ylabel("Robustness", fontsize = 16)
axs[1, 0].set_xticks([0, 1])
axs[1, 0].set_yticklabels(['', ''], weight = "bold",fontsize = 15)  # Generic labels
axs[1, 0].set_xticklabels([t_0_label, t_f_label], weight = "bold",fontsize = 15)  # Generic labelsaxs[1, 0].set_yticks([])
axs[1, 0].legend()
# axs[1, 0].grid(True, linestyle="--", alpha=0.6)

# Binary Cost
# Binary Cost Plot
signal_binary = np.where(np.any(duration > 0), 1, 0) * np.ones_like(time)  # Binary step signal
axs[1, 1].step(time, duration, where="mid", color="blue", linewidth=2, label="Signal")  # Cost
axs[1, 1].plot(time, signal_binary, color="r", label="Cost")  # Signal
axs[1, 1].set_title("Binary Cost", weight="bold", fontsize=15)
axs[1, 1].set_xlabel("Time", fontsize=16)
axs[1, 1].set_ylabel("Robustness Indicator", fontsize=16)
axs[1, 1].set_yticks([0, 1])
axs[1, 1].set_xticks([0, 1])
axs[1, 1].set_yticklabels(["0", "1"], weight="bold", fontsize=15)
axs[1, 1].set_xticklabels([t_0_label, t_f_label], weight="bold", fontsize=15)
axs[1, 1].legend(fontsize=12)

# axs[1, 1].grid(True, linestyle="--", alpha=0.6)

# Display the plots
# plt.suptitle("Visualization of Cost Function Designs (Generic Representation)", fontsize=14)
plt.show()




# import numpy as np
# import matplotlib.pyplot as plt

# # Define the time range
# time = np.linspace(0, 1, 100)  # Normalized time from 0 to 1
# t_f_label = r"$t_f$"  # Final time label
# t_0_label = r"$t_0$"  # Initial time label

# # Define cost functions
# duration = np.where((time > 0.2) & (time < 0.7), 1, 0)  # Binary duration
# severity = np.where((time > 0.3) & (time < 0.7), np.sin((time - 0.3) * np.pi / 0.4), 0)  # Severity signal
# severity[severity < 0] = 0  # Ensure non-negative values
# duration_severity = severity  # Area under the curve for duration-severity

# # Define signals
# signal_duration = np.zeros_like(time)
# signal_duration[duration == 1] = 1  # Positive during cost=1
# signal_duration[time <= 0.2] = -0.5  # Negative before cost jumps to 1
# signal_duration[time >= 0.7] = -0.5  # Negative after cost returns to 0

# signal_severity = severity  # Use the same as severity
# signal_binary = np.where(np.any(duration > 0), 1, 0) * np.ones_like(time)  # Binary step signal

# # Set up the figure and axes
# fig, axs = plt.subplots(2, 2, figsize=(12, 8))
# fig.subplots_adjust(hspace=0.4, wspace=0.3)

# # Duration-Based Cost Plot
# axs[0, 0].plot(time, signal_duration, color="orange", linewidth=2, label="Signal")  # Signal
# axs[0, 0].step(time, duration, where="mid", color="skyblue", linestyle="--", linewidth=2, label="Cost")  # Cost
# axs[0, 0].fill_between(time, duration, color="lightblue", alpha=0.8, step="mid")
# axs[0, 0].set_title("Duration-Based Cost", weight="bold", fontsize=15)
# axs[0, 0].set_xlabel("Time", fontsize=16)
# axs[0, 0].set_ylabel("Value", fontsize=16)
# axs[0, 0].set_yticks([-0.5, 0, 1])
# axs[0, 0].set_xticks([0, 1])
# axs[0, 0].set_yticklabels(["-0.5", "0", "1"], weight="bold", fontsize=15)
# axs[0, 0].set_xticklabels([t_0_label, t_f_label], weight="bold", fontsize=15)
# axs[0, 0].legend(fontsize=12)

# # Duration-Severity Cost Plot
# axs[0, 1].plot(time, signal_severity, color="purple", linewidth=2, label="Signal")  # Signal
# axs[0, 1].plot(time, duration_severity, color="plum", linestyle="--", linewidth=2, label="Cost")  # Cost
# axs[0, 1].fill_between(time, duration_severity, color="plum", alpha=0.7)
# axs[0, 1].set_title("Duration-Severity Cost", weight="bold", fontsize=15)
# axs[0, 1].set_xlabel("Time", fontsize=16)
# axs[0, 1].set_ylabel("Value", fontsize=16)
# axs[0, 1].set_xticks([0, 1])
# axs[0, 1].set_yticks([0, 1])
# axs[0, 1].set_yticklabels(["0", "1"], weight="bold", fontsize=15)
# axs[0, 1].set_xticklabels([t_0_label, t_f_label], weight="bold", fontsize=15)
# axs[0, 1].legend(fontsize=12)

# # Severity-Based Cost Plot
# axs[1, 0].plot(time, signal_severity, color="red", linewidth=2, label="Signal")  # Signal
# axs[1, 0].axhline(np.max(severity), color="darkred", linestyle="--", linewidth=2, label="Cost")  # Cost
# axs[1, 0].set_title("Severity-Based Cost", weight="bold", fontsize=15)
# axs[1, 0].set_xlabel("Time", fontsize=16)
# axs[1, 0].set_ylabel("Value", fontsize=16)
# axs[1, 0].set_xticks([0, 1])
# axs[1, 0].set_yticks([0, 1])
# axs[1, 0].set_yticklabels(["0", "1"], weight="bold", fontsize=15)
# axs[1, 0].set_xticklabels([t_0_label, t_f_label], weight="bold", fontsize=15)
# axs[1, 0].legend(fontsize=12)

# # Binary Cost Plot
# axs[1, 1].plot(time, signal_binary, color="black", linewidth=2, label="Signal")  # Signal
# axs[1, 1].step(time, duration, where="mid", color="gray", linestyle="--", linewidth=2, label="Cost")  # Cost
# axs[1, 1].fill_between(time, duration, color="lightgray", alpha=0.5, step="mid")
# axs[1, 1].set_title("Binary Cost", weight="bold", fontsize=15)
# axs[1, 1].set_xlabel("Time", fontsize=16)
# axs[1, 1].set_ylabel("Value", fontsize=16)
# axs[1, 1].set_yticks([0, 1])
# axs[1, 1].set_xticks([0, 1])
# axs[1, 1].set_yticklabels(["0", "1"], weight="bold", fontsize=15)
# axs[1, 1].set_xticklabels([t_0_label, t_f_label], weight="bold", fontsize=15)
# axs[1, 1].legend(fontsize=12)

# # Display the plots
# plt.show()
