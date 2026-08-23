"""Deterministic, capacity-constrained message transport.

The radio graph answers whether a physical link exists.  This module models
whether information actually crosses that link.  It deliberately knows
nothing about the environment or ground truth.
"""

from __future__ import annotations

import hashlib
import json
from collections import defaultdict, deque
from dataclasses import dataclass
from enum import Enum
from typing import Iterable

from echorescue.communication import CommunicationLink, CommunicationSnapshot


class MessageType(str, Enum):
    MOTION_INTENT = "motion_intent"
    DRONE_STATE = "drone_state"
    SURVIVOR_DETECTION = "survivor_detection"
    SURVIVOR_CONFIRMATION = "survivor_confirmation"
    MAP_UPDATE = "map_update"
    TELEMETRY = "telemetry"


MESSAGE_PRIORITY = {
    MessageType.MOTION_INTENT: 0,
    MessageType.DRONE_STATE: 1,
    MessageType.SURVIVOR_CONFIRMATION: 2,
    MessageType.SURVIVOR_DETECTION: 3,
    MessageType.MAP_UPDATE: 4,
    MessageType.TELEMETRY: 5,
}


class TransmissionStatus(str, Enum):
    QUEUED = "queued"
    IN_FLIGHT = "in_flight"
    DELIVERED = "delivered"
    EXPIRED = "expired"
    DROPPED = "dropped"


@dataclass(slots=True)
class NetworkFragment:
    message_id: str
    fragment_id: str
    sender: str
    recipient: str
    route: tuple[str, ...]
    message_type: MessageType
    created_step: int
    ttl: int
    priority: int
    payload: tuple[object, ...]
    payload_units: int
    fragment_index: int
    fragment_count: int
    current_hop: int = 0
    attempt: int = 0
    available_step: int = 0
    sent_step: int | None = None
    earliest_delivery_step: int | None = None
    status: TransmissionStatus = TransmissionStatus.QUEUED

    @property
    def next_link(self) -> tuple[str, str] | None:
        if self.current_hop >= len(self.route) - 1:
            return None
        return self.route[self.current_hop], self.route[self.current_hop + 1]

    @property
    def expires_after_step(self) -> int:
        return self.created_step + self.ttl


@dataclass(frozen=True, slots=True)
class NetworkDelivery:
    message_id: str
    fragment_id: str
    sender: str
    recipient: str
    message_type: MessageType
    payload: tuple[object, ...]
    fragment_index: int
    fragment_count: int
    created_step: int
    delivered_step: int
    route: tuple[str, ...]

    @property
    def latency(self) -> int:
        return self.delivered_step - self.created_step

    @property
    def relayed(self) -> bool:
        return len(self.route) > 2


@dataclass(frozen=True, slots=True)
class NetworkTransportEvent:
    event_type: str
    step: int
    sender: str
    recipient: str
    message_type: MessageType
    message_id: str
    fragment_count: int = 1
    payload_units: int = 0
    link: tuple[str, str] | None = None


def shortest_route(
    snapshot: CommunicationSnapshot,
    sender: str,
    recipient: str,
) -> tuple[str, ...]:
    """Return a stable shortest route over physical links."""

    if sender == recipient:
        return (sender,)
    adjacency = {node_id: [] for node_id in snapshot.nodes}
    for link in snapshot.links:
        adjacency[link.first].append(link.second)
        adjacency[link.second].append(link.first)
    if sender not in adjacency or recipient not in adjacency:
        return ()
    queue = deque([(sender, (sender,))])
    visited = {sender}
    while queue:
        node, route = queue.popleft()
        for neighbor in sorted(adjacency[node]):
            if neighbor in visited:
                continue
            next_route = route + (neighbor,)
            if neighbor == recipient:
                return next_route
            visited.add(neighbor)
            queue.append((neighbor, next_route))
    return ()


def _canonical(value: object) -> object:
    if isinstance(value, Enum):
        return value.value
    if hasattr(value, "x") and hasattr(value, "y"):
        return [getattr(value, "x"), getattr(value, "y")]
    if hasattr(value, "__dataclass_fields__"):
        return {
            name: _canonical(getattr(value, name))
            for name in value.__dataclass_fields__
        }
    if isinstance(value, dict):
        return {
            str(key): _canonical(item)
            for key, item in sorted(value.items(), key=lambda pair: str(pair[0]))
        }
    if isinstance(value, (tuple, list, set, frozenset)):
        items = [_canonical(item) for item in value]
        return sorted(items, key=repr) if isinstance(value, (set, frozenset)) else items
    return value


class DeterministicNetworkTransport:
    """A store-and-forward transport with deterministic loss and fairness."""

    def __init__(
        self,
        *,
        seed: int,
        profile: str,
        latency_steps: int,
        packet_loss_rate: float,
        link_capacity_units: int,
        max_fragment_units: int,
        fairness_age_steps: int,
    ) -> None:
        if latency_steps < 1:
            raise ValueError("network latency must be at least one step")
        if not 0.0 <= packet_loss_rate < 1.0:
            raise ValueError("packet loss rate must be in [0, 1)")
        if link_capacity_units < 1 or max_fragment_units < 1:
            raise ValueError("network capacities must be positive")
        if max_fragment_units > link_capacity_units:
            raise ValueError("a fragment must fit within one link-step")
        if fairness_age_steps < 1:
            raise ValueError("fairness age must be positive")
        self.seed = seed
        self.profile = profile
        self.latency_steps = latency_steps
        self.packet_loss_rate = packet_loss_rate
        self.link_capacity_units = link_capacity_units
        self.max_fragment_units = max_fragment_units
        self.fairness_age_steps = fairness_age_steps
        self._queued: list[NetworkFragment] = []
        self._in_flight: list[NetworkFragment] = []
        self._active_message_ids: set[str] = set()
        self._delivered_fragments: dict[str, set[int]] = defaultdict(set)
        self._message_fragment_counts: dict[str, int] = {}
        self._created_fragment_ids: set[str] = set()
        self._delivered_fragment_ids: set[str] = set()
        self._events: list[NetworkTransportEvent] = []
        self._events_cursor = 0
        self._queue_samples: list[int] = []
        self._backlog_started: dict[str, int] = {}
        self.transmission_attempts = 0
        self.successful_transmission_attempts = 0
        self.retransmission_attempts = 0
        self.queued_messages = 0
        self.sent_fragments = 0
        self.delivered_fragments = 0
        self.delivered_messages = 0
        self.lost_fragments = 0
        self.expired_fragments = 0
        self.dropped_fragments = 0
        self.payload_units_delivered = 0
        self.relay_fragments_forwarded = 0
        self.stale_intents = 0
        self.delivery_latencies: list[int] = []
        self.relay_latencies: list[int] = []
        self.max_backlog_duration = 0
        self.routes_replanned = 0
        self.rerouting_enabled = True

    @property
    def queue_size(self) -> int:
        return len(self._queued) + len(self._in_flight)

    @property
    def average_queue_size(self) -> float:
        return (
            sum(self._queue_samples) / len(self._queue_samples)
            if self._queue_samples
            else 0.0
        )

    @property
    def maximum_queue_size(self) -> int:
        return max(self._queue_samples, default=0)

    @property
    def delivery_ratio(self) -> float:
        """Backward-compatible alias for unique-fragment eventual delivery."""

        return self.unique_fragment_eventual_delivery_ratio

    @property
    def fragment_attempt_delivery_ratio(self) -> float:
        """Successful link-hop attempts divided by all link-hop attempts."""

        return (
            self.successful_transmission_attempts / self.transmission_attempts
            if self.transmission_attempts
            else 1.0
        )

    @property
    def unique_fragment_eventual_delivery_ratio(self) -> float:
        """Unique end fragments delivered divided by unique fragments created."""

        return (
            len(self._delivered_fragment_ids) / len(self._created_fragment_ids)
            if self._created_fragment_ids
            else 1.0
        )

    @property
    def logical_message_completion_ratio(self) -> float:
        """Completed logical messages divided by logical messages queued."""

        return (
            self.delivered_messages / self.queued_messages
            if self.queued_messages
            else 1.0
        )

    @property
    def created_fragments(self) -> int:
        return len(self._created_fragment_ids)

    @property
    def mean_latency(self) -> float:
        return (
            sum(self.delivery_latencies) / len(self.delivery_latencies)
            if self.delivery_latencies
            else 0.0
        )

    @property
    def max_latency(self) -> int:
        return max(self.delivery_latencies, default=0)

    def drain_events(self) -> tuple[NetworkTransportEvent, ...]:
        events = tuple(self._events[self._events_cursor :])
        self._events_cursor = len(self._events)
        return events

    def message_is_active(self, message_id: str) -> bool:
        """Return whether at least one fragment of a message is still pending."""

        return message_id in self._active_message_ids

    def _message_identity(
        self,
        *,
        sender: str,
        recipient: str,
        message_type: MessageType,
        created_step: int,
        payload: tuple[object, ...],
        message_key: str | None,
    ) -> str:
        identity = {
            "seed": self.seed,
            "profile": self.profile,
            "sender": sender,
            "recipient": recipient,
            "message_type": message_type.value,
            "created_step": created_step,
            "key": message_key,
            "payload": _canonical(payload),
        }
        digest = hashlib.sha256(
            json.dumps(identity, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()[:20]
        return f"msg-{digest}"

    def enqueue(
        self,
        *,
        sender: str,
        recipient: str,
        route: tuple[str, ...],
        message_type: MessageType,
        payload: Iterable[object],
        created_step: int,
        ttl: int,
        message_key: str | None = None,
        units_per_item: int = 1,
    ) -> str | None:
        payload = tuple(payload)
        if not payload or not route or route[0] != sender or route[-1] != recipient:
            return None
        if ttl < 1 or units_per_item < 1:
            raise ValueError("message TTL and item size must be positive")
        message_id = self._message_identity(
            sender=sender,
            recipient=recipient,
            message_type=message_type,
            created_step=created_step,
            payload=payload,
            message_key=message_key,
        )
        if message_id in self._active_message_ids:
            return None
        items_per_fragment = max(1, self.max_fragment_units // units_per_item)
        chunks = [
            payload[index : index + items_per_fragment]
            for index in range(0, len(payload), items_per_fragment)
        ]
        self._active_message_ids.add(message_id)
        self.queued_messages += 1
        self._message_fragment_counts[message_id] = len(chunks)
        for index, chunk in enumerate(chunks):
            units = len(chunk) * units_per_item
            fragment = NetworkFragment(
                message_id=message_id,
                fragment_id=f"{message_id}:f{index}",
                sender=sender,
                recipient=recipient,
                route=route,
                message_type=message_type,
                created_step=created_step,
                ttl=ttl,
                priority=MESSAGE_PRIORITY[message_type],
                payload=chunk,
                payload_units=units,
                fragment_index=index,
                fragment_count=len(chunks),
                available_step=created_step,
            )
            self._queued.append(fragment)
            self._created_fragment_ids.add(fragment.fragment_id)
            self._backlog_started[fragment.fragment_id] = created_step
        self._events.append(
            NetworkTransportEvent(
                event_type="message_queued",
                step=created_step,
                sender=sender,
                recipient=recipient,
                message_type=message_type,
                message_id=message_id,
                fragment_count=len(chunks),
                payload_units=sum(fragment.payload_units for fragment in self._queued if fragment.message_id == message_id),
            )
        )
        return message_id

    def _is_lost(self, fragment: NetworkFragment, step: int) -> bool:
        link = fragment.next_link
        assert link is not None
        identity = "|".join(
            (
                str(self.seed),
                self.profile,
                link[0],
                link[1],
                fragment.fragment_id,
                str(fragment.attempt),
                str(step),
            )
        )
        value = int.from_bytes(hashlib.sha256(identity.encode()).digest()[:8], "big")
        return value / 2**64 < self.packet_loss_rate

    def _expire(self, step: int) -> None:
        expired_message_ids: set[str] = set()
        for collection in (self._queued, self._in_flight):
            survivors = []
            for fragment in collection:
                if step <= fragment.expires_after_step:
                    survivors.append(fragment)
                    continue
                fragment.status = TransmissionStatus.EXPIRED
                self.expired_fragments += 1
                expired_message_ids.add(fragment.message_id)
                if fragment.message_type is MessageType.MOTION_INTENT:
                    self.stale_intents += 1
                self.max_backlog_duration = max(
                    self.max_backlog_duration,
                    step - self._backlog_started.pop(fragment.fragment_id, step),
                )
                self._events.append(
                    NetworkTransportEvent(
                        event_type="message_expired",
                        step=step,
                        sender=fragment.sender,
                        recipient=fragment.recipient,
                        message_type=fragment.message_type,
                        message_id=fragment.message_id,
                        fragment_count=1,
                        payload_units=fragment.payload_units,
                        link=fragment.next_link,
                    )
                )
            collection[:] = survivors
        pending_message_ids = {
            fragment.message_id for fragment in self._queued + self._in_flight
        }
        self._active_message_ids.difference_update(
            expired_message_ids - pending_message_ids
        )

    def _deliver_due(self, step: int) -> list[NetworkDelivery]:
        delivered: list[NetworkDelivery] = []
        remaining = []
        for fragment in sorted(
            self._in_flight,
            key=lambda item: (
                item.earliest_delivery_step or 0,
                item.fragment_id,
                item.current_hop,
            ),
        ):
            if (fragment.earliest_delivery_step or 0) > step:
                remaining.append(fragment)
                continue
            link = fragment.next_link
            assert link is not None
            fragment.current_hop += 1
            if fragment.current_hop < len(fragment.route) - 1:
                fragment.status = TransmissionStatus.QUEUED
                fragment.available_step = step
                fragment.sent_step = None
                fragment.earliest_delivery_step = None
                self._queued.append(fragment)
                self.relay_fragments_forwarded += 1
                self._events.append(
                    NetworkTransportEvent(
                        event_type="relay_message_forwarded",
                        step=step,
                        sender=link[0],
                        recipient=link[1],
                        message_type=fragment.message_type,
                        message_id=fragment.message_id,
                        payload_units=fragment.payload_units,
                        link=link,
                    )
                )
                continue
            fragment.status = TransmissionStatus.DELIVERED
            self.delivered_fragments += 1
            self._delivered_fragment_ids.add(fragment.fragment_id)
            self.payload_units_delivered += fragment.payload_units
            latency = step - fragment.created_step
            self.delivery_latencies.append(latency)
            if len(fragment.route) > 2:
                self.relay_latencies.append(latency)
            self.max_backlog_duration = max(
                self.max_backlog_duration,
                step - self._backlog_started.pop(fragment.fragment_id, step),
            )
            self._delivered_fragments[fragment.message_id].add(fragment.fragment_index)
            complete = len(self._delivered_fragments[fragment.message_id]) == fragment.fragment_count
            if complete:
                self.delivered_messages += 1
                self._active_message_ids.discard(fragment.message_id)
            delivered.append(
                NetworkDelivery(
                    message_id=fragment.message_id,
                    fragment_id=fragment.fragment_id,
                    sender=fragment.sender,
                    recipient=fragment.recipient,
                    message_type=fragment.message_type,
                    payload=fragment.payload,
                    fragment_index=fragment.fragment_index,
                    fragment_count=fragment.fragment_count,
                    created_step=fragment.created_step,
                    delivered_step=step,
                    route=fragment.route,
                )
            )
            self._events.append(
                NetworkTransportEvent(
                    event_type=(
                        "message_fragment_completed"
                        if complete and fragment.fragment_count > 1
                        else "message_delivered"
                    ),
                    step=step,
                    sender=fragment.sender,
                    recipient=fragment.recipient,
                    message_type=fragment.message_type,
                    message_id=fragment.message_id,
                    fragment_count=fragment.fragment_count if complete else 1,
                    payload_units=fragment.payload_units,
                    link=link,
                )
            )
        self._in_flight = remaining
        return delivered

    def _effective_priority(self, fragment: NetworkFragment, step: int) -> int:
        age_boost = max(0, step - fragment.created_step) // self.fairness_age_steps
        return max(0, fragment.priority - age_boost)

    def _transmit(self, step: int, links: set[CommunicationLink]) -> None:
        candidates: dict[CommunicationLink, list[NetworkFragment]] = defaultdict(list)
        for fragment in self._queued:
            link = fragment.next_link
            if link is None or fragment.available_step > step:
                continue
            normalized = CommunicationLink.between(*link)
            if normalized in links:
                candidates[normalized].append(fragment)
        sent_ids: set[str] = set()
        for normalized_link in sorted(candidates):
            capacity = self.link_capacity_units
            ordered = sorted(
                candidates[normalized_link],
                key=lambda fragment: (
                    self._effective_priority(fragment, step),
                    fragment.created_step,
                    fragment.priority,
                    fragment.fragment_id,
                    fragment.current_hop,
                    fragment.attempt,
                ),
            )
            for fragment in ordered:
                if fragment.payload_units > capacity:
                    continue
                capacity -= fragment.payload_units
                sent_ids.add(fragment.fragment_id)
                self.transmission_attempts += 1
                self.sent_fragments += 1
                if fragment.attempt:
                    self.retransmission_attempts += 1
                fragment.sent_step = step
                link = fragment.next_link
                assert link is not None
                if self._is_lost(fragment, step):
                    self.lost_fragments += 1
                    fragment.attempt += 1
                    fragment.available_step = step + 1
                    self._events.append(
                        NetworkTransportEvent(
                            event_type="message_lost",
                            step=step,
                            sender=link[0],
                            recipient=link[1],
                            message_type=fragment.message_type,
                            message_id=fragment.message_id,
                            payload_units=fragment.payload_units,
                            link=link,
                        )
                    )
                    continue
                self.successful_transmission_attempts += 1
                fragment.status = TransmissionStatus.IN_FLIGHT
                fragment.earliest_delivery_step = step + self.latency_steps
                self._in_flight.append(fragment)
        self._queued = [
            fragment
            for fragment in self._queued
            if fragment.fragment_id not in sent_ids or fragment.status is TransmissionStatus.QUEUED
        ]

    def advance(
        self,
        *,
        step: int,
        snapshot: CommunicationSnapshot,
    ) -> tuple[NetworkDelivery, ...]:
        """Expire, deliver, and transmit once for a simulation step."""

        deliveries = self.deliver(step=step)
        self.transmit(step=step, snapshot=snapshot)
        return deliveries

    def deliver(self, *, step: int) -> tuple[NetworkDelivery, ...]:
        """Deliver only fragments whose configured latency elapsed."""

        self._expire(step)
        return tuple(self._deliver_due(step))

    def transmit(
        self,
        *,
        step: int,
        snapshot: CommunicationSnapshot,
    ) -> None:
        """Schedule queued fragments over currently valid physical links."""

        self._expire(step)
        if self.rerouting_enabled:
            self._reroute_queued(snapshot)
        self._transmit(step, set(snapshot.links))
        self._queue_samples.append(self.queue_size)

    def _reroute_queued(self, snapshot: CommunicationSnapshot) -> None:
        """Repair invalid next hops from the currently observable graph only.

        This is deliberately not predictive routing.  A fragment is reconsidered
        only when its stored next hop is no longer a current physical link.
        """

        links = set(snapshot.links)
        for fragment in sorted(
            self._queued,
            key=lambda item: (item.fragment_id, item.current_hop),
        ):
            link = fragment.next_link
            if link is None or CommunicationLink.between(*link) in links:
                continue
            current_node = fragment.route[fragment.current_hop]
            replacement = shortest_route(
                snapshot, current_node, fragment.recipient
            )
            if not replacement:
                continue
            fragment.route = (
                fragment.route[: fragment.current_hop] + replacement
            )
            self.routes_replanned += 1

    def finalize(self, step: int) -> None:
        """Mark unfinished fragments dropped for a closed mission."""

        for fragment in sorted(
            self._queued + self._in_flight,
            key=lambda item: (item.fragment_id, item.current_hop),
        ):
            fragment.status = TransmissionStatus.DROPPED
            self.dropped_fragments += 1
            self.max_backlog_duration = max(
                self.max_backlog_duration,
                step - self._backlog_started.pop(fragment.fragment_id, step),
            )
            self._events.append(
                NetworkTransportEvent(
                    event_type="message_dropped",
                    step=step,
                    sender=fragment.sender,
                    recipient=fragment.recipient,
                    message_type=fragment.message_type,
                    message_id=fragment.message_id,
                    payload_units=fragment.payload_units,
                    link=fragment.next_link,
                )
            )
        self._queued.clear()
        self._in_flight.clear()
