"""Webots 3D visualization and presentation configuration model."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class WebotsPresentationConfig:
    """Configuration for Webots presentation playback, camera, and overlay layers.

    Attributes:
        sub_steps: Visual interpolation intermediate frames per authoritative 1.0s tick.
        sim_mode: Webots simulation execution mode ('default', 'fast', 'realtime').
        default_camera: Name of default Webots viewpoint camera node.
        show_comm_mesh: Whether active RF communication mesh lines are rendered.
        show_routes: Whether active multi-hop routes to GCS are rendered.
        show_drop_lines: Whether altitude ground plumb-lines and footprints are rendered.
        show_hud: Whether in-world HUD telemetry overlay is displayed.
        show_event_banner: Whether top event notification alert banners are displayed.
        auto_quit: Whether Webots automatically terminates upon scenario completion.
    """

    sub_steps: int = 4
    sim_mode: str = "default"
    default_camera: str = "demo_presentation_cam"
    show_comm_mesh: bool = True
    show_routes: bool = True
    show_drop_lines: bool = True
    show_hud: bool = True
    show_event_banner: bool = True
    auto_quit: bool = False
    playback_speed: float = 1.0
    camera_mode: str = "overview"
    show_pois: bool = True
    show_grid: bool = True
    header_title: str = "AETHERSWARM UAV-X AUTONOMOUS WORKING MODEL"

    def to_dict(self) -> dict[str, Any]:
        return {
            "sub_steps": self.sub_steps,
            "sim_mode": self.sim_mode,
            "default_camera": self.default_camera,
            "show_comm_mesh": self.show_comm_mesh,
            "show_routes": self.show_routes,
            "show_drop_lines": self.show_drop_lines,
            "show_hud": self.show_hud,
            "show_event_banner": self.show_event_banner,
            "auto_quit": self.auto_quit,
            "playback_speed": self.playback_speed,
            "camera_mode": self.camera_mode,
            "show_pois": self.show_pois,
            "show_grid": self.show_grid,
            "header_title": self.header_title,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> WebotsPresentationConfig:
        return cls(
            sub_steps=int(data.get("sub_steps", 4)),
            sim_mode=str(data.get("sim_mode", "default")),
            default_camera=str(data.get("default_camera", "demo_presentation_cam")),
            show_comm_mesh=bool(data.get("show_comm_mesh", True)),
            show_routes=bool(data.get("show_routes", True)),
            show_drop_lines=bool(data.get("show_drop_lines", True)),
            show_hud=bool(data.get("show_hud", True)),
            show_event_banner=bool(data.get("show_event_banner", True)),
            auto_quit=bool(data.get("auto_quit", False)),
            playback_speed=float(data.get("playback_speed", 1.0)),
            camera_mode=str(data.get("camera_mode", "overview")),
            show_pois=bool(data.get("show_pois", True)),
            show_grid=bool(data.get("show_grid", True)),
            header_title=str(data.get("header_title", "AETHERSWARM UAV-X AUTONOMOUS WORKING MODEL")),
        )
