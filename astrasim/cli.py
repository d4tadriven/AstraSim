import typer
from rich.console import Console
from astrasim.demos import aircraft_stability, rocket_launch, custom_component

app = typer.Typer(help="AstraSim: Elite Aerospace Simulation Platform")
console = Console()

@app.command()
def run(simulation: str):
    """
    Run a specific simulation: 'aircraft_stability', 'rocket_launch', or 'custom_component'
    """
    if simulation == "aircraft_stability":
        aircraft_stability.run_simulation()
    elif simulation == "rocket_launch":
        rocket_launch.run_simulation()
    elif simulation == "custom_component":
        custom_component.run_simulation()
    else:
        console.print(f"[bold red]Unknown simulation:[/bold red] {simulation}")

@app.command()
def export(simulation: str, format: str = typer.Option("gif", "--format", "-f", help="Export format (gif, mp4)")):
    """
    Run and export a simulation
    """
    if simulation == "aircraft_stability":
        aircraft_stability.run_simulation(export_format=format)
    elif simulation == "rocket_launch":
        rocket_launch.run_simulation(export_format=format)
    elif simulation == "custom_component":
        custom_component.run_simulation(export_format=format)
    else:
        console.print(f"[bold red]Unknown simulation:[/bold red] {simulation}")

if __name__ == "__main__":
    app()
