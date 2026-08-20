from collections import deque
from dataclasses import dataclass

from echorescue.environment import GridWorld
from echorescue.models import Position


BASE_NODE_ID = "base"


def _line_cells(start: Position, end: Position) -> tuple[Position, ...]:
    """Return a deterministic conservative supercover line."""

    x, y = start.x, start.y
    dx = end.x - start.x
    dy = end.y - start.y
    nx = abs(dx)
    ny = abs(dy)
    sign_x = 0 if dx == 0 else (1 if dx > 0 else -1)
    sign_y = 0 if dy == 0 else (1 if dy > 0 else -1)
    ix = 0
    iy = 0
    cells = [start]
    while ix < nx or iy < ny:
        decision = (1 + 2 * ix) * ny - (1 + 2 * iy) * nx
        if decision == 0:
            cells.append(Position(x + sign_x, y))
            cells.append(Position(x, y + sign_y))
            x += sign_x
            y += sign_y
            ix += 1
            iy += 1
        elif decision < 0:
            x += sign_x
            ix += 1
        else:
            y += sign_y
            iy += 1
        cells.append(Position(x, y))
    return tuple(dict.fromkeys(cells))


@dataclass(frozen=True, order=True, slots=True)
class CommunicationLink:
    first: str
    second: str

    @classmethod
    def between(cls, first: str, second: str) -> "CommunicationLink":
        return cls(*sorted((first, second)))


@dataclass(frozen=True, slots=True)
class DroneConnection:
    connected_to_base: bool
    direct_to_base: bool
    relay_path: tuple[str, ...] = ()

    @property
    def via_relay(self) -> bool:
        return self.connected_to_base and not self.direct_to_base


@dataclass(frozen=True, slots=True)
class CommunicationSnapshot:
    nodes: dict[str, Position]
    links: tuple[CommunicationLink, ...]
    connections: dict[str, DroneConnection]

    def has_link(self, first: str, second: str) -> bool:
        return CommunicationLink.between(first, second) in self.links


@dataclass(frozen=True, slots=True)
class CommunicationModel:
    max_range: int

    def __post_init__(self) -> None:
        if self.max_range < 1:
            raise ValueError("communication range must be positive")

    def _has_line_of_sight(
        self, world: GridWorld, first: Position, second: Position
    ) -> bool:
        intervening = _line_cells(first, second)[1:-1]
        return all(world.is_free(position) for position in intervening)

    def _can_link(
        self, world: GridWorld, first: Position, second: Position
    ) -> bool:
        distance_squared = (first.x - second.x) ** 2 + (first.y - second.y) ** 2
        return (
            distance_squared <= self.max_range**2
            and self._has_line_of_sight(world, first, second)
        )

    @staticmethod
    def _path_to_base(
        drone_id: str,
        adjacency: dict[str, tuple[str, ...]],
    ) -> tuple[str, ...]:
        queue = deque([(BASE_NODE_ID, (BASE_NODE_ID,))])
        visited = {BASE_NODE_ID}
        while queue:
            node, path = queue.popleft()
            if node == drone_id:
                return path
            for neighbor in adjacency[node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + (neighbor,)))
        return ()

    def compute(
        self,
        world: GridWorld,
        base: Position,
        drone_positions: dict[str, Position],
    ) -> CommunicationSnapshot:
        nodes = {BASE_NODE_ID: base, **dict(sorted(drone_positions.items()))}
        node_ids = sorted(nodes)
        links = []
        adjacency_lists = {node_id: [] for node_id in node_ids}
        for index, first_id in enumerate(node_ids):
            for second_id in node_ids[index + 1 :]:
                if self._can_link(world, nodes[first_id], nodes[second_id]):
                    link = CommunicationLink.between(first_id, second_id)
                    links.append(link)
                    adjacency_lists[first_id].append(second_id)
                    adjacency_lists[second_id].append(first_id)
        adjacency = {
            node_id: tuple(sorted(neighbors))
            for node_id, neighbors in adjacency_lists.items()
        }
        connections = {}
        for drone_id in sorted(drone_positions):
            path = self._path_to_base(drone_id, adjacency)
            direct = CommunicationLink.between(BASE_NODE_ID, drone_id) in links
            connections[drone_id] = DroneConnection(
                connected_to_base=bool(path),
                direct_to_base=direct,
                relay_path=path if path and not direct else (),
            )
        return CommunicationSnapshot(
            nodes=nodes,
            links=tuple(sorted(links)),
            connections=connections,
        )
