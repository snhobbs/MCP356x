import rich
from rich.table import Table
from rich import console
from rich.console import Console
from rich.pretty import pprint
from . import objects
import numpy as np

def generate_settings_table(registers) -> Table:
    """Make a new table."""""
    table = Table(title="MCP35xx Settings", box=rich.box.MINIMAL_DOUBLE_HEAD)
    table.add_column("Field")
    table.add_column("Setting")

    t = objects.internal_registers_to_configuration_table(registers)

    for key, value in t._settings.items():
        row = [key]
        row.append(str(value))
        table.add_row(*np.array(row, dtype=str))
    return table

