"""Get commands for stepper motor."""
from stepper.stepper_core.configs import Code, StatusCode  # <--- 增加 StatusCode
from stepper.commands.commands import (
    Command,
    Protocol,
    ReturnData,
    TakeNoSetting,
    WithNoParams,
)
from stepper.stepper_core.configs import Code
from stepper.stepper_core.parameters import (
    BusVoltageParams,
    ConfigParams,
    EncoderParams,
    MotorRHParams,
    OpenLoopTargetPositionParams,
    PhaseCurrentParams,
    PIDParams,
    PositionErrorParams,
    PulseCountParams,
    RealTimePositionParams,
    RealTimeSpeedParams,
    StepperStatus,
    SystemParams,
    TargetPositionParams,
    VersionParams,
)

__all__ = [
    "GetVersion",
    "GetMotorRH",
    "GetPID",
    "GetBusVoltage",
    "GetPhaseCurrent",
    "GetEncoderValue",
    "GetPulseCount",
    "GetTargetPosition",
    "GetOpenLoopSetpoint",
    "GetRealTimeSpeed",
    "GetRealTimePosition",
    "GetPositionError",
    "GetStatus",
    "GetConfig",
    "GetSysStatus",
]


class GetCommand(WithNoParams, TakeNoSetting, ReturnData, Command):
    """Get command configuration."""


class GetVersion(GetCommand):
    """Get version of the device with adaptive length support."""

    _code = Code.GET_VERSION
    # _response_length 在此逻辑中不再固定依赖，但为了兼容性保留默认值
    _response_length = 5 
    ReturnType = VersionParams

    def _execute(self) -> StatusCode:
        """
        重写执行逻辑以兼容 Emm42(5字节) 和 Y42(7字节)。
        策略：先读5字节，检查校验位；如果不是0x6B，则认为是Y42，补读剩余字节。
        """
        # 1. 清空缓冲区并发送命令
        self.serial_connection.reset_input_buffer()
        self.serial_connection.reset_output_buffer()
        self.serial_connection.write(self._command)
        
        try:
            # 2. 读取头部 (地址 + 功能码) = 2字节
            header = self.serial_connection.read(2)
            if len(header) < 2:
                return StatusCode.MAX_RETRIES_EXCEEDED # 超时

            # 3. 尝试按旧款 Emm42 读取 (2字节数据 + 1字节校验)
            # 如果是 Emm42: [D0, D1, Checksum(0x6B)]
            # 如果是 Y42:   [D0, D1, D2(硬件信息)]
            chunk1 = self.serial_connection.read(3)
            if len(chunk1) < 3:
                return StatusCode.ERROR

            # 4. 判断策略：检查第5个字节是否为固定的校验位 0x6B (b'k')
            # Y42 的 D2 通常是 0x23 (#) 或其他值，极大概率不是 0x6B
            is_v5_protocol = (chunk1[-1] == 0x6B)

            if is_v5_protocol:
                # ---> 是旧款 Emm42 (5字节)
                data = chunk1[:-1] # 提取2字节数据
            else:
                # ---> 可能是新款 Y42 (7字节)，需要再读 2 个字节
                # Y42剩余: [D3, Checksum]
                chunk2 = self.serial_connection.read(2)
                if len(chunk2) < 2:
                    return StatusCode.ERROR
                
                # 再次验证最终的校验位
                if chunk2[-1] != 0x6B:
                    # 如果补读后还不是 0x6B，说明通讯真出错了
                    return StatusCode.ERROR
                
                # 拼接完整数据：chunk1全部(3字节) + chunk2第一个字节(1字节) = 4字节数据
                data = chunk1 + chunk2[:-1]

            # 5. 调用父类的数据处理逻辑 (解析 VersionParams)
            # 注意：你的 VersionParams.from_bytes 必须能同时处理 2字节 和 4字节 的输入
            return self._process_data(data)

        except Exception:
            return StatusCode.ERROR


class GetMotorRH(GetCommand):
    """Get motor resistance and inductance."""

    _code = Code.GET_MOTOR_R_H
    _response_length = 7
    ReturnType = MotorRHParams


class GetPID(GetCommand):
    """Get PID parameters command configuration."""

    _code = Code.GET_PID
    _response_length = 15
    ReturnType = PIDParams


class GetBusVoltage(GetCommand):
    """Get the bus voltage."""

    _code = Code.GET_BUS_VOLTAGE
    _response_length = 5
    ReturnType = BusVoltageParams


class GetPhaseCurrent(GetCommand):
    """Get phase current."""

    _code = Code.GET_PHASE_CURRENT
    _response_length = 5
    ReturnType = PhaseCurrentParams


class GetEncoderValue(GetCommand):
    """Get encoder value."""

    _code = Code.GET_ENCODER_VALUE
    _response_length = 5
    ReturnType = EncoderParams


class GetPulseCount(GetCommand):
    """Get pulse count."""

    _code = Code.GET_PULSE_COUNT
    _response_length = 8
    ReturnType = PulseCountParams


class GetTargetPosition(GetCommand):
    """Get target position."""

    _code = Code.GET_TARGET
    _response_length = 8
    ReturnType = TargetPositionParams


class GetOpenLoopSetpoint(GetCommand):
    """Get open loop setpoint."""

    _code = Code.GET_OPEN_LOOP_SETPOINT
    _response_length = 8
    ReturnType = OpenLoopTargetPositionParams


class GetRealTimeSpeed(GetCommand):
    """Get real time speed in RPM."""

    _code = Code.GET_SPEED
    _response_length = 6
    ReturnType = RealTimeSpeedParams


class GetRealTimePosition(GetCommand):
    """Get real time position command configuration."""

    _code = Code.GET_POS
    _response_length = 8
    ReturnType = RealTimePositionParams


class GetPositionError(GetCommand):
    """Get error command configuration."""

    _code = Code.GET_ERROR
    _response_length = 8
    ReturnType = PositionErrorParams


class GetStatus(GetCommand):
    """Get status of the stepper motor."""

    _code = Code.GET_STATUS
    _response_length = 4
    ReturnType = StepperStatus


class GetConfig(GetCommand):
    """Get configuration."""

    _code = Code.GET_CONFIG
    _protocol = Protocol.GET_CONFIG
    _response_length = 33
    ReturnType = ConfigParams


class GetSysStatus(GetCommand):
    """Get system status command configuration."""

    _code = Code.GET_SYS_STATUS
    _protocol = Protocol.GET_SYS_STATUS
    _response_length = 31
    ReturnType = SystemParams
