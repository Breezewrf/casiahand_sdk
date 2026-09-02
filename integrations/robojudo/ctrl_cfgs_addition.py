"""Copy this class into ``robojudo/controller/ctrl_cfgs.py``."""

from pydantic import Field, model_validator
from robojudo.config import Config


class CasiaHandCfg(Config):
    """Direct dual CASIA Hand-M hardware control."""

    left_hand_id: int = Field(default=2, ge=0, le=255)
    right_hand_id: int = Field(default=0x20, ge=0, le=255)
    baudrate: int = Field(default=115200, gt=0)
    port_name: str = "/dev/ttyUSB0"
    command_timeout_s: float = Field(default=0.25, gt=0.0)
    joint_state_fps: float = Field(default=100.0, gt=0.0)
    joint_state_timeout_s: float = Field(default=0.25, gt=0.0)
    startup_timeout_s: float = Field(default=5.0, gt=0.0)

    @model_validator(mode="after")
    def validate_casia_hand(self):
        if not self.port_name.strip():
            raise ValueError("CASIA serial port_name must not be empty")
        if self.left_hand_id == self.right_hand_id:
            raise ValueError("CASIA left and right hand IDs must differ")
        return self
