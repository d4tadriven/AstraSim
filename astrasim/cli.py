import typer
from rich.console import Console
from astrasim.demos import aircraft_stability

app = typer.Typer(help="AstraSim: Elite Aerospace Simulation Platform")
console = Console()

@app.command()
def run(simulation: str):
    """
    Run a specific simulation, e.g., 'aircraft_stability'
    """
    if simulation == "aircraft_stability":
        aircraft_stability.run_simulation()
    else:
        console.print(f"[bold red]Unknown simulation:[/bold red] {simulation}")

@app.command()
def export(simulation: str, format: str = typer.Option("gif", "--format", "-f", help="Export format (gif, mp4)")):
    """
    Run and export a simulation
    """
    if simulation == "aircraft_stability":
        aircraft_stability.run_simulation(export_format=format)
    else:
        console.print(f"[bold red]Unknown simulation:[/bold red] {simulation}")

if __name__ == "__main__":
    app()
