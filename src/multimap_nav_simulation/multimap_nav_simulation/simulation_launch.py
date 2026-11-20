"""Startup event utility retained for existing external launch imports."""

from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def after_success(action, following):
    def on_exit(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason=f"Simulation startup failed: exit {event.returncode}"))]
        return following
    return RegisterEventHandler(OnProcessExit(target_action=action, on_exit=on_exit))
