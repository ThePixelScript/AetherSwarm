"""Autonomy package for ARES-Swarm."""

from .task_allocator import (
    A0TaskAllocator,
    AllocationResult,
    AllocationWeights,
    PositionProtocol,
    TaskActionProposal,
    TaskAllocator,
    TaskAllocatorConfig,
    TaskAssignment,
    UtilityScore,
)

from .a1_allocator import A1CommunicationAwareAllocator

__all__ = [
    "A0TaskAllocator",
    "A1CommunicationAwareAllocator",
    "AllocationResult",
    "AllocationWeights",
    "PositionProtocol",
    "TaskActionProposal",
    "TaskAllocator",
    "TaskAllocatorConfig",
    "TaskAssignment",
    "UtilityScore",
]
