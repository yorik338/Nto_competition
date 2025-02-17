from dataclasses import dataclass
from enum import auto, Enum

import numpy as np

from Robot2 import move_box_by_y
from Robot_1 import move_box_by_x
from camera import scan

BOX_SIZE = 10  # Длина стороны коробки


@dataclass
class Box:
    x: float
    y: float
    id: int
    countur: np.ndarray


class CellType(Enum):
    input = auto()
    memory = auto()
    output = auto()


@dataclass
class Cell:
    x: float
    y: float
    id: int
    type: CellType


input_area_cells, boxes = scan()


def find_most_down_available_point(main_box_x):
    for i in range(len(boxes)):
        box_x = boxes[i].x
        if abs(box_x - main_box_x) < BOX_SIZE:
            return boxes[i].y - BOX_SIZE / 2
    return


def main():
    def choose_memory_cell(box: Box) -> Cell:
        pass

    def find_optimal_available_coord_for_box(
            box: Box,
            box_size: int,
            axis: str,
            direction: int,
            map_data: np.ndarray[np.ndarray]
    ) -> int | None:
        assert axis in 'xy'
        assert direction in [1, -1]

        if axis == 'x':
            map_slice = map_data[
                        box.y - box_size / 2:box.y + box_size / 2 + 1,
                        slice(box.x + box_size / 2, map_data.shape[1]) if direction == 1 else
                        slice(0, box.x - box_size / 2, -1)
                        ]
        else:
            map_slice = map_data[
                        box.x - box_size / 2:box.x + box_size / 2 + 1,
                        slice(box.y + box_size / 2, map_data.shape[0]) if direction == 1 else
                        slice(0, box.y - box_size / 2, -1)
                        ]

        map_slice_compressed = np.any(map_slice != 255, axis=0 if axis == 'x' else 1)

        axis_local_index = None
        for axis_local_index, axis_result in enumerate(map_slice_compressed):
            if not axis_result:
                break

        if axis_local_index is None:
            return None

        if axis == 'x':
            index_delta = (
                (box.x + box_size / 2) if direction == 1 else
                (box.x - box_size / 2)
            )
        else:
            index_delta = (
                (box.y + box_size / 2) if direction == 1 else
                (box.y - box_size / 2)
            )

        return round(index_delta + axis_local_index) if direction == 1 else round(index_delta - axis_local_index)

    for box in input_area_cells:  # type: Box
        memory_cell_for_box = choose_memory_cell(box)

        target_x_for_box = min(
            memory_cell_for_box.x,
            find_optimal_available_coord_for_box(
                box,
                BOX_SIZE,
                'x',
                1,
            )
        )
        move_box_by_x(target_x_for_box)

        if abs(box.y - memory_cell_for_box.y) > 10:
            if (box.y - memory_cell_for_box.y) < 0:
                target_y_for_box = min(
                    memory_cell_for_box.y,
                    find_optimal_available_coord_for_box(
                        box,
                        BOX_SIZE,
                        'y',
                        1,
                    )
                )
            else:
                target_y_for_box = max(
                    memory_cell_for_box.y,
                    find_optimal_available_coord_for_box(
                        box,
                        BOX_SIZE,
                        'y',
                        -1,
                    )
                )

            move_box_by_y(target_y_for_box)

        if abs(box.x - memory_cell_for_box.x) > 10:  # мб стоит обратить это в цикл
            move_box_by_x(memory_cell_for_box.x)
