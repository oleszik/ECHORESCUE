from dataclasses import dataclass

from echorescue.communication import BASE_NODE_ID, CommunicationSnapshot
from echorescue.knowledge import KnowledgeMap, merge_knowledge_maps
from echorescue.models import Position


@dataclass(frozen=True, slots=True)
class MapSyncReport:
    uploaded_by_drone: dict[str, int]
    received_by_drone: dict[str, int]
    base_cells_received: int
    transfer_occurred: bool
    sync_available: bool
    transferred_positions: frozenset[Position]
    semantic_cell_changes: int


class ShadowMapSynchronizer:
    """Synchronize local knowledge over the observed communication graph."""

    def __init__(
        self,
        local_maps: dict[str, KnowledgeMap],
        base_map: KnowledgeMap | None,
    ) -> None:
        self.local_maps = dict(sorted(local_maps.items()))
        self.base_map = base_map

    @staticmethod
    def connected_components(
        snapshot: CommunicationSnapshot,
    ) -> tuple[tuple[str, ...], ...]:
        adjacency = {node_id: set() for node_id in snapshot.nodes}
        for link in snapshot.links:
            adjacency[link.first].add(link.second)
            adjacency[link.second].add(link.first)
        components = []
        remaining = set(adjacency)
        while remaining:
            start = min(remaining)
            stack = [start]
            component = set()
            while stack:
                node = stack.pop()
                if node in component:
                    continue
                component.add(node)
                remaining.discard(node)
                stack.extend(sorted(adjacency[node] - component, reverse=True))
            components.append(tuple(sorted(component)))
        return tuple(components)

    def sync(self, snapshot: CommunicationSnapshot) -> MapSyncReport:
        stores = dict(self.local_maps)
        if self.base_map is not None:
            stores[BASE_NODE_ID] = self.base_map
        uploaded = {drone_id: set() for drone_id in self.local_maps}
        received = {drone_id: set() for drone_id in self.local_maps}
        base_received: set[Position] = set()
        transferred_positions: set[Position] = set()
        semantic_cell_changes = 0
        sync_available = False

        for component in self.connected_components(snapshot):
            participants = [node_id for node_id in component if node_id in stores]
            if len(participants) < 2:
                continue
            sync_available = True
            merged = merge_knowledge_maps(stores[node_id] for node_id in participants)
            merged_records = dict(merged.records)
            for target_id in participants:
                target = stores[target_id]
                semantic_changes = {
                    position
                    for position, record in merged.records
                    if target.cell_at(position) is not record.state
                }
                changed = target.apply(merged.records)
                transferred_positions.update(changed)
                semantic_cell_changes += len(semantic_changes.intersection(changed))
                if target_id in received:
                    received[target_id].update(changed)
                elif target_id == BASE_NODE_ID:
                    base_received.update(changed)
                for position in changed:
                    source_id = merged_records[position].source_id
                    if (
                        source_id in participants
                        and source_id in uploaded
                        and source_id != target_id
                    ):
                        uploaded[source_id].add(position)

        uploaded_counts = {
            drone_id: len(positions)
            for drone_id, positions in uploaded.items()
        }
        received_counts = {
            drone_id: len(positions)
            for drone_id, positions in received.items()
        }
        transfer_occurred = bool(
            len(base_received)
            or any(uploaded_counts.values())
            or any(received_counts.values())
        )
        return MapSyncReport(
            uploaded_by_drone=uploaded_counts,
            received_by_drone=received_counts,
            base_cells_received=len(base_received),
            transfer_occurred=transfer_occurred,
            sync_available=sync_available,
            transferred_positions=frozenset(transferred_positions),
            semantic_cell_changes=semantic_cell_changes,
        )

    def shared_shadow_map(self) -> KnowledgeMap:
        return merge_knowledge_maps(self.local_maps.values())

    def maps_converged(self) -> bool:
        maps = tuple(self.local_maps.values())
        return not maps[0].differs_from(maps[1])

    def divergence_ratio(self) -> float:
        maps = tuple(self.local_maps.values())
        return len(maps[0].differs_from(maps[1])) / (
            maps[0].width * maps[0].height
        )
