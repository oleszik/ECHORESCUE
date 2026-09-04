from dataclasses import dataclass
from enum import Enum

from echorescue.models import Position


ROLE_POLICIES = {"off", "generalist", "generalized"}
TASK_LOAD_PENALTY = 4
GENERALIST_MISMATCH_PENALTY = 3
SPECIALIST_MISMATCH_PENALTY = 6
ENERGY_PENALTY_INTERVAL = 20.0


class AgentRole(str, Enum):
    GENERALIST = "GENERALIST"
    SCOUT = "SCOUT"
    VERIFIER = "VERIFIER"
    RELAY = "RELAY"


class TaskType(str, Enum):
    EXPLORATION = "exploration"
    SURVIVOR_VERIFICATION = "survivor_verification"
    RELAY_POSITIONING = "relay_positioning"
    RETURN_HOME = "return_home"


class TaskStatus(str, Enum):
    ASSIGNED = "assigned"
    ORPHANED = "orphaned"
    COMPLETED = "completed"
    CANCELLED = "cancelled"


@dataclass(slots=True)
class MissionTask:
    identifier: str
    task_type: TaskType
    owner_id: str | None
    status: TaskStatus
    target: Position
    created_step: int
    priority: int
    original_owner_id: str
    orphaned_step: int | None = None
    reassigned_step: int | None = None
    execution_recovered_step: int | None = None
    completed_step: int | None = None

    @property
    def reassigned(self) -> bool:
        return self.reassigned_step is not None

    def to_dict(self) -> dict[str, object]:
        payload: dict[str, object] = {
            "task_id": self.identifier,
            "task_type": self.task_type.value,
            "owner_agent_id": self.owner_id,
            "status": self.status.value,
            "target": [self.target.x, self.target.y],
            "created_step": self.created_step,
            "priority": self.priority,
            "original_owner_agent_id": self.original_owner_id,
        }
        if self.orphaned_step is not None:
            payload["orphaned_step"] = self.orphaned_step
        if self.reassigned_step is not None:
            payload["reassigned_step"] = self.reassigned_step
        if self.execution_recovered_step is not None:
            payload["execution_recovered_step"] = (
                self.execution_recovered_step
            )
        if self.completed_step is not None:
            payload["completed_step"] = self.completed_step
        return payload


class TaskRegistry:
    """Deterministic single-owner mission-task registry."""

    def __init__(self) -> None:
        self.tasks: dict[str, MissionTask] = {}
        self._active_by_owner: dict[str, str] = {}
        self._next_identifier = 1

    def current_for(self, owner_id: str) -> MissionTask | None:
        task_id = self._active_by_owner.get(owner_id)
        return self.tasks.get(task_id) if task_id is not None else None

    def create(
        self,
        *,
        owner_id: str,
        task_type: TaskType,
        target: Position,
        step: int,
        priority: int,
    ) -> MissionTask:
        current = self.current_for(owner_id)
        if (
            current is not None
            and current.task_type is task_type
            and current.target == target
            and current.status is TaskStatus.ASSIGNED
        ):
            return current
        if current is not None:
            self.cancel_owner(owner_id, step)
        identifier = f"task-{self._next_identifier:06d}"
        self._next_identifier += 1
        task = MissionTask(
            identifier=identifier,
            task_type=task_type,
            owner_id=owner_id,
            status=TaskStatus.ASSIGNED,
            target=target,
            created_step=step,
            priority=priority,
            original_owner_id=owner_id,
        )
        self.tasks[identifier] = task
        self._active_by_owner[owner_id] = identifier
        return task

    def orphan_owner(self, owner_id: str, step: int) -> MissionTask | None:
        task = self.current_for(owner_id)
        if task is None:
            return None
        self._active_by_owner.pop(owner_id, None)
        task.owner_id = None
        task.status = TaskStatus.ORPHANED
        task.orphaned_step = step
        return task

    def assign_orphan(
        self,
        task_id: str,
        *,
        owner_id: str,
        target: Position,
        step: int,
    ) -> MissionTask:
        task = self.tasks[task_id]
        if task.status is not TaskStatus.ORPHANED or task.owner_id is not None:
            raise ValueError("only an orphaned task can be reassigned")
        current = self.current_for(owner_id)
        if current is not None:
            self.cancel_owner(owner_id, step)
        task.owner_id = owner_id
        task.target = target
        task.status = TaskStatus.ASSIGNED
        task.reassigned_step = step
        self._active_by_owner[owner_id] = task.identifier
        return task

    def complete_owner(self, owner_id: str, step: int) -> MissionTask | None:
        task = self.current_for(owner_id)
        if task is None:
            return None
        self._active_by_owner.pop(owner_id, None)
        task.status = TaskStatus.COMPLETED
        task.completed_step = step
        return task

    def cancel_owner(self, owner_id: str, step: int) -> MissionTask | None:
        task = self.current_for(owner_id)
        if task is None:
            return None
        self._active_by_owner.pop(owner_id, None)
        task.status = TaskStatus.CANCELLED
        task.completed_step = step
        return task

    def active_or_orphaned(self) -> tuple[MissionTask, ...]:
        return tuple(
            task
            for _, task in sorted(self.tasks.items())
            if task.status in {TaskStatus.ASSIGNED, TaskStatus.ORPHANED}
        )

    def active_load(self, owner_id: str) -> int:
        return int(self.current_for(owner_id) is not None)


def initial_roles(
    agent_ids: tuple[str, ...], policy: str
) -> dict[str, AgentRole]:
    ordered = tuple(sorted(agent_ids))
    if policy in {"off", "generalist"} or len(ordered) == 1:
        return {agent_id: AgentRole.GENERALIST for agent_id in ordered}
    scout_count = max(1, len(ordered) // 2)
    verifier_count = max(1, len(ordered) // 4)
    roles: dict[str, AgentRole] = {}
    for index, agent_id in enumerate(ordered):
        if index < scout_count:
            role = AgentRole.SCOUT
        elif index < scout_count + verifier_count:
            role = AgentRole.VERIFIER
        else:
            role = AgentRole.GENERALIST
        roles[agent_id] = role
    return roles


def preferred_role(task_type: TaskType) -> AgentRole:
    if task_type is TaskType.EXPLORATION:
        return AgentRole.SCOUT
    if task_type is TaskType.SURVIVOR_VERIFICATION:
        return AgentRole.VERIFIER
    if task_type is TaskType.RELAY_POSITIONING:
        return AgentRole.RELAY
    return AgentRole.GENERALIST


def role_mismatch_penalty(role: AgentRole, task_type: TaskType) -> int:
    preferred = preferred_role(task_type)
    if role is preferred:
        return 0
    if role is AgentRole.GENERALIST or task_type is TaskType.RETURN_HOME:
        return GENERALIST_MISMATCH_PENALTY
    return SPECIALIST_MISMATCH_PENALTY
