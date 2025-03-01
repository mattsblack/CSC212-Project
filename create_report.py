#!/usr/bin/env python3
"""
This script reads the CSV files generated from an ROS2 bag export
and creates an HTML report with data previews, descriptive statistics,
and example plots. It does not modify the original CSV files.
 
Dependencies:
  - pandas
  - matplotlib
  - os
  - pathlib
Make sure to install any missing packages via pip.
"""

import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

# List of CSV files with a title for the report.
DATA_FILES = [
    ("Battery Data", "battery_data.csv"),
    ("Wheel Status (Custom)", "wheel_status_custom.csv"),
    ("Interface Buttons", "interface_buttons.csv"),
    ("Hazard Detection", "hazard_detection.csv"),
    ("LIDAR Scan", "lidar_scan.csv"),
    ("Odometry Data", "odometry_data.csv"),
    ("Wheel Status (Simple)", "wheel_status_simple.csv"),
]

REPORT_DIR = Path("report_output")
REPORT_DIR.mkdir(exist_ok=True)

def save_plot(plt_obj, filename):
    """Save the current plot to the report directory and clear the figure."""
    file_path = REPORT_DIR / filename
    plt_obj.savefig(file_path, bbox_inches="tight")
    plt_obj.clf()
    return file_path.name  # Return just the filename for HTML embedding

def generate_html_report():
    report_path = REPORT_DIR / "data_report.html"
    with report_path.open("w") as html:
        html.write("<html><head><title>ROS2 Bag Data Report</title>")
        html.write("<style>table { border-collapse: collapse; } th, td { border: 1px solid #ccc; padding: 4px; }</style>")
        html.write("</head><body>")
        html.write("<h1>ROS2 Bag Data Report</h1>")
        
        # Loop over each data file.
        for title, filename in DATA_FILES:
            file_path = Path(filename)
            if not file_path.exists():
                html.write(f"<h2>{title}</h2>")
                html.write(f"<p><em>File not found: {filename}</em></p>")
                continue

            # Read the CSV file.
            try:
                df = pd.read_csv(file_path)
            except Exception as e:
                html.write(f"<h2>{title}</h2>")
                html.write(f"<p><em>Error reading file: {e}</em></p>")
                continue

            html.write(f"<h2>{title} ({filename})</h2>")
            html.write("<h3>Data Preview (first 10 rows)</h3>")
            html.write(df.head(10).to_html(index=False, classes="data-table"))
            
            # Add descriptive statistics if applicable (numeric data).
            numeric_cols = df.select_dtypes(include=["number"]).columns
            if not numeric_cols.empty:
                html.write("<h3>Descriptive Statistics</h3>")
                html.write(df.describe().to_html(classes="data-table"))
            
            # Generate sample plots for some of the files.
            plot_filename = None
            if filename == "battery_data.csv" and {"timestamp", "voltage", "percentage"}.issubset(df.columns):
                plt.figure()
                df.plot(x="timestamp", y=["voltage", "percentage"], title="Battery Data Over Time")
                plot_filename = save_plot(plt, "battery_data_plot.png")
            elif filename == "odometry_data.csv" and {"timestamp", "pos_x", "pos_y", "pos_z"}.issubset(df.columns):
                plt.figure()
                df.plot(x="timestamp", y=["pos_x", "pos_y", "pos_z"], title="Odometry Position Over Time")
                plot_filename = save_plot(plt, "odometry_data_plot.png")
            elif filename == "wheel_status_custom.csv" and {"timestamp", "current_ma_left", "current_ma_right"}.issubset(df.columns):
                plt.figure()
                df.plot(x="timestamp", y=["current_ma_left", "current_ma_right"], title="Wheel Current Over Time")
                plot_filename = save_plot(plt, "wheel_current_plot.png")
            
            if plot_filename:
                html.write("<h3>Plot</h3>")
                html.write(f'<img src="{plot_filename}" alt="Plot for {title}" style="max-width:800px;">')
            
            html.write("<hr>")  # Separate sections
        html.write("</body></html>")
    
    print(f"HTML report generated: {report_path.resolve()}")

if __name__ == "__main__":
    generate_html_report()
