import json
import os
from rich.console import Console
from rich.table import Table

CONFIG_FILE = "mission_config.json"
console = Console()

DEFAULT_CONFIG = {
    "min_confidence": 0.5,
    "center_threshold_m": 0.15,
    "descent_rate": 0.5,
    "drop_altitude": 3.0,
    "camera_fov": 62.2,
    "target_lat": 0.0,
    "target_lon": 0.0,
    "servo_channel": 1
}

def load_config():
    if not os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, 'w') as f: json.dump(DEFAULT_CONFIG, f, indent=4)
    with open(CONFIG_FILE, 'r') as f: return json.load(f)

def save_config(config):
    with open(CONFIG_FILE, 'w') as f: json.dump(config, f, indent=4)

def update_menu():
    conf = load_config()
    table = Table(title="Mission Parameters")
    table.add_column("Index", style="cyan")
    table.add_column("Parameter", style="white")
    table.add_column("Value", style="magenta")
    
    keys = list(conf.keys())
    for i, key in enumerate(keys):
        table.add_row(str(i), key, str(conf[key]))
    
    console.print(table)
    choice = console.input("\nSelect [bold cyan]Index[/bold cyan] to edit (or 'q' to exit): ")
    
    if choice.isdigit() and int(choice) < len(keys):
        key = keys[int(choice)]
        new_val = float(console.input(f"Enter new value for {key}: "))
        conf[key] = new_val
        save_config(conf)
        console.print(f"[green]Updated {key} to {new_val}[/green]")