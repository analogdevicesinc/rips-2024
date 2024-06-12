################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################

from abc import ABC

class Field:
    def __init__(self, address, width, mask, shift, name, *, signed=False, value=None, parent=None):
        self.ADDRESS = address
        self.WIDTH = width
        self.WIDTH_MASK = (1 << (width * 8)) - 1
        self.MASK = mask
        self.SHIFT = shift
        self.VALUE = value

        self.SIGNED=signed

        self._PARENT = parent

    def get_name(self):
        parent = self._PARENT

        for item in parent.__dir__():
            if getattr(parent, item) == self:
                return f"{parent.get_name()}.{item}"

        raise RuntimeError("Field name could not be constructed")

    def write(self, value):
        return (value << self.SHIFT) & self.MASK

    def read(self, value):
        value = (value & self.MASK) >> self.SHIFT
        if self.SIGNED:
            base_mask = self.MASK >> self.SHIFT
            sign_mask = base_mask & (~base_mask >> 1)
            value = (value ^ sign_mask) - sign_mask
        return value

    def update(self, read_register, value):
        return (read_register & (self.WIDTH_MASK ^ self.MASK)) | self.write(value)

    def is_in_bounds(self, value):
        base_mask = self.MASK >> self.SHIFT
        if self.SIGNED:
            return -base_mask//2 <= value <= base_mask//2
        else:
            return 0 <= value <= base_mask

    def get_address_block(self):
        return self._PARENT._PARENT

class Register(ABC):
    """
    Abstract base class for register defined in map folders.

    The main purpose is to give these classes an easy way to set the value of a field for use with the bulk write functionality of the reg module.
    """
    def find(self, name: str):
        field = getattr(self, name)
        if not field or not isinstance(field, Field):
            raise ValueError(f"Register {self.get_name()} does not have a field named {name}")

        return field

    def get_name(self):
        parent = self._PARENT

        for item in parent.__dir__():
            if getattr(parent, item) == self:
                return f"{parent._NAME}.{item}"

        raise RuntimeError("Register name could not be constructed")

    @property
    def SIGNED(self):
        field_list = self.get_fields()
        if len(field_list) > 1:
            # Multiple fields -> treat the compound register as unsigned
            return False

        # Just one field -> treat the register with the same signedness as the field
        return field_list[0].SIGNED

    def __setattr__(self, __name: str, __value: any) -> None:
        """
        This function is called when doing Register.Field = value
        """

        if hasattr(self, __name) and isinstance(__value, int) and isinstance(getattr(self, __name), Field):
            # If the Register attribute being accessed is a Field: i.e Register.Field = value
            # And if the value provided is an integer.
            # Then we write Register.Field.VALUE with __value.
            setattr(getattr(self, __name), "VALUE", __value)
        else:
            # Else we attribute like we'd usually do.
            super().__setattr__(__name, __value)

    def fields(self, *args):
        """
        Assign field values to Register object

        Arguments should follow the following pattern: Register.Field, int_value, Register.Field, int_value...
        """

        if len(args) % 2 != 0:
            raise ValueError("Args length should be a multiple of 2, always a field and then a value for that field")
        if len(args) != 0 and not isinstance(args[0], Field):
            raise ValueError("Args should start with a Field and alternate between Field and Int.")

        for index in range(0, len(args), 2):
            if not isinstance(args[index + 1], int):
                raise ValueError("Args not formatted correctly, should alternate Field and Int")
            args[index].VALUE = args[index + 1]

        return self

    def get_fields(self):
        """
        Returns a list of all the Field members of this Register object
        """
        fields = []
        for key in self.__dir__():

            # We have to skip the special SIGNED key here, to avoid infinite recursion
            if key == "SIGNED":
                continue

            obj = getattr(self, key)
            if isinstance(obj, Field):
                fields.append(obj)

        return fields

    def get_address_block(self):
        return self._PARENT

class AddressBlock(ABC):
    """
    This base class represents an AddressBlock, containing a list of registers.
    It also contains convenience functions.
    """

    def find(self, name: str):
        segments = name.split(".")

        if len(segments) == 0:
            raise ValueError("Invalid name")

        if segments[0] != self._NAME:
            raise ValueError("Invalid name: AddressBlock name doesn't match")

        if len(segments) == 1:
            return self

        # We have at least two segments -> search for the register
        register = getattr(self, segments[1], None)
        if not register:
            raise ValueError(f"Invalid name: Register {segments[1]} not found in {self._NAME}")

        if len(segments) == 2:
            return register

        # We have at least three segments -> search for field in register
        field = getattr(register, segments[2])
        if not field:
            raise ValueError(f"Invalid name '{name}': Field {segments[2]} not found in register {register.get_name()}")

        return field

    def get_registers(self):
        """
        Returns a list of all the Register members of this AddressBlock object
        """
        regs = []
        for key in self.__dir__():

            obj = getattr(self, key)
            if isinstance(obj, Register):
                regs.append(obj)

        return regs


class _tm01():
    def __init__(self):
        self.ADC = _Adc()
        self.AON = _Aon()
        self.CLKCTRL = _ClkCtrl()
        self.CONFIG = _Config()
        self.CORDIC = _Cordic()
        self.GPIO = _Gpio()
        self.I2C = _I2c()
        self.IOMATRIX = _IoMatrix()
        self.MCC = _Mcc()
        self.OTPCTRL = _OtpCtrl()
        self.PIC = _Pic()
        self.SPI0 = _Spi0()
        self.SPI1 = _Spi1()
        self.SPISLAVE = _SpiSlave()
        self.SYSCTRL = _SysCtrl()
        self.TIMADV = _TimAdv()
        self.TIMBASIC0 = _TimBasic0()
        self.TIMBASIC1 = _TimBasic1()
        self.TIMBASIC2 = _TimBasic2()
        self.TIMSYS = _TimSys()
        self.UART = _Uart()
        self.WDT = _Wdt()

class _Gpio(AddressBlock):
    _NAME       = "GPIO"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.DATA           = _Gpio_Data(self)
        self.SET            = _Gpio_Set(self)
        self.CLEAR          = _Gpio_Clear(self)
        self.TOGGLE         = _Gpio_Toggle(self)
        self.DIR            = _Gpio_Dir(self)
        self.IRQ_POLARITY   = _Gpio_IrqPolarity(self)
        self.IRQ_MASK       = _Gpio_IrqMask(self)
        self.IRQ_STATUS     = _Gpio_IrqStatus(self)
        self.IRQ_BOTH_EDGES = _Gpio_IrqBothEdges(self)
        self.IRQ_EDGE_LEVEL = _Gpio_IrqEdgeLevel(self)


class _Gpio_Data(Register):
    ADDRESS = 0x48002000
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PIN_0  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "GPIO.DATA.PIN_0", signed=False, parent=self)
        self.PIN_1  = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "GPIO.DATA.PIN_1", signed=False, parent=self)
        self.PIN_2  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "GPIO.DATA.PIN_2", signed=False, parent=self)
        self.PIN_3  = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "GPIO.DATA.PIN_3", signed=False, parent=self)
        self.PIN_4  = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "GPIO.DATA.PIN_4", signed=False, parent=self)
        self.PIN_5  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "GPIO.DATA.PIN_5", signed=False, parent=self)
        self.PIN_6  = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "GPIO.DATA.PIN_6", signed=False, parent=self)
        self.PIN_7  = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "GPIO.DATA.PIN_7", signed=False, parent=self)
        self.PIN_8  = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "GPIO.DATA.PIN_8", signed=False, parent=self)
        self.PIN_9  = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "GPIO.DATA.PIN_9", signed=False, parent=self)
        self.PIN_10 = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "GPIO.DATA.PIN_10", signed=False, parent=self)
        self.PIN_11 = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "GPIO.DATA.PIN_11", signed=False, parent=self)
        self.PIN_12 = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "GPIO.DATA.PIN_12", signed=False, parent=self)
        self.PIN_13 = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "GPIO.DATA.PIN_13", signed=False, parent=self)
        self.PIN_14 = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "GPIO.DATA.PIN_14", signed=False, parent=self)
        self.PIN_15 = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "GPIO.DATA.PIN_15", signed=False, parent=self)
        self.PIN_16 = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "GPIO.DATA.PIN_16", signed=False, parent=self)
        self.PIN_17 = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "GPIO.DATA.PIN_17", signed=False, parent=self)
        self.PIN_18 = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "GPIO.DATA.PIN_18", signed=False, parent=self)

class _Gpio_Set(Register):
    ADDRESS = 0x48002004
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PIN_0  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "GPIO.SET.PIN_0", signed=False, parent=self)
        self.PIN_1  = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "GPIO.SET.PIN_1", signed=False, parent=self)
        self.PIN_2  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "GPIO.SET.PIN_2", signed=False, parent=self)
        self.PIN_3  = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "GPIO.SET.PIN_3", signed=False, parent=self)
        self.PIN_4  = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "GPIO.SET.PIN_4", signed=False, parent=self)
        self.PIN_5  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "GPIO.SET.PIN_5", signed=False, parent=self)
        self.PIN_6  = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "GPIO.SET.PIN_6", signed=False, parent=self)
        self.PIN_7  = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "GPIO.SET.PIN_7", signed=False, parent=self)
        self.PIN_8  = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "GPIO.SET.PIN_8", signed=False, parent=self)
        self.PIN_9  = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "GPIO.SET.PIN_9", signed=False, parent=self)
        self.PIN_10 = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "GPIO.SET.PIN_10", signed=False, parent=self)
        self.PIN_11 = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "GPIO.SET.PIN_11", signed=False, parent=self)
        self.PIN_12 = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "GPIO.SET.PIN_12", signed=False, parent=self)
        self.PIN_13 = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "GPIO.SET.PIN_13", signed=False, parent=self)
        self.PIN_14 = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "GPIO.SET.PIN_14", signed=False, parent=self)
        self.PIN_15 = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "GPIO.SET.PIN_15", signed=False, parent=self)
        self.PIN_16 = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "GPIO.SET.PIN_16", signed=False, parent=self)
        self.PIN_17 = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "GPIO.SET.PIN_17", signed=False, parent=self)
        self.PIN_18 = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "GPIO.SET.PIN_18", signed=False, parent=self)

class _Gpio_Clear(Register):
    ADDRESS = 0x48002008
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PIN_0  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "GPIO.CLEAR.PIN_0", signed=False, parent=self)
        self.PIN_1  = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "GPIO.CLEAR.PIN_1", signed=False, parent=self)
        self.PIN_2  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "GPIO.CLEAR.PIN_2", signed=False, parent=self)
        self.PIN_3  = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "GPIO.CLEAR.PIN_3", signed=False, parent=self)
        self.PIN_4  = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "GPIO.CLEAR.PIN_4", signed=False, parent=self)
        self.PIN_5  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "GPIO.CLEAR.PIN_5", signed=False, parent=self)
        self.PIN_6  = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "GPIO.CLEAR.PIN_6", signed=False, parent=self)
        self.PIN_7  = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "GPIO.CLEAR.PIN_7", signed=False, parent=self)
        self.PIN_8  = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "GPIO.CLEAR.PIN_8", signed=False, parent=self)
        self.PIN_9  = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "GPIO.CLEAR.PIN_9", signed=False, parent=self)
        self.PIN_10 = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "GPIO.CLEAR.PIN_10", signed=False, parent=self)
        self.PIN_11 = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "GPIO.CLEAR.PIN_11", signed=False, parent=self)
        self.PIN_12 = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "GPIO.CLEAR.PIN_12", signed=False, parent=self)
        self.PIN_13 = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "GPIO.CLEAR.PIN_13", signed=False, parent=self)
        self.PIN_14 = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "GPIO.CLEAR.PIN_14", signed=False, parent=self)
        self.PIN_15 = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "GPIO.CLEAR.PIN_15", signed=False, parent=self)
        self.PIN_16 = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "GPIO.CLEAR.PIN_16", signed=False, parent=self)
        self.PIN_17 = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "GPIO.CLEAR.PIN_17", signed=False, parent=self)
        self.PIN_18 = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "GPIO.CLEAR.PIN_18", signed=False, parent=self)

class _Gpio_Toggle(Register):
    ADDRESS = 0x4800200C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PIN_0  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "GPIO.TOGGLE.PIN_0", signed=False, parent=self)
        self.PIN_1  = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "GPIO.TOGGLE.PIN_1", signed=False, parent=self)
        self.PIN_2  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "GPIO.TOGGLE.PIN_2", signed=False, parent=self)
        self.PIN_3  = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "GPIO.TOGGLE.PIN_3", signed=False, parent=self)
        self.PIN_4  = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "GPIO.TOGGLE.PIN_4", signed=False, parent=self)
        self.PIN_5  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "GPIO.TOGGLE.PIN_5", signed=False, parent=self)
        self.PIN_6  = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "GPIO.TOGGLE.PIN_6", signed=False, parent=self)
        self.PIN_7  = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "GPIO.TOGGLE.PIN_7", signed=False, parent=self)
        self.PIN_8  = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "GPIO.TOGGLE.PIN_8", signed=False, parent=self)
        self.PIN_9  = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "GPIO.TOGGLE.PIN_9", signed=False, parent=self)
        self.PIN_10 = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "GPIO.TOGGLE.PIN_10", signed=False, parent=self)
        self.PIN_11 = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "GPIO.TOGGLE.PIN_11", signed=False, parent=self)
        self.PIN_12 = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "GPIO.TOGGLE.PIN_12", signed=False, parent=self)
        self.PIN_13 = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "GPIO.TOGGLE.PIN_13", signed=False, parent=self)
        self.PIN_14 = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "GPIO.TOGGLE.PIN_14", signed=False, parent=self)
        self.PIN_15 = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "GPIO.TOGGLE.PIN_15", signed=False, parent=self)
        self.PIN_16 = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "GPIO.TOGGLE.PIN_16", signed=False, parent=self)
        self.PIN_17 = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "GPIO.TOGGLE.PIN_17", signed=False, parent=self)
        self.PIN_18 = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "GPIO.TOGGLE.PIN_18", signed=False, parent=self)

class _Gpio_Dir(Register):
    ADDRESS = 0x48002010
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PIN_0  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "GPIO.DIR.PIN_0", signed=False, parent=self)
        self.PIN_1  = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "GPIO.DIR.PIN_1", signed=False, parent=self)
        self.PIN_2  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "GPIO.DIR.PIN_2", signed=False, parent=self)
        self.PIN_3  = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "GPIO.DIR.PIN_3", signed=False, parent=self)
        self.PIN_4  = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "GPIO.DIR.PIN_4", signed=False, parent=self)
        self.PIN_5  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "GPIO.DIR.PIN_5", signed=False, parent=self)
        self.PIN_6  = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "GPIO.DIR.PIN_6", signed=False, parent=self)
        self.PIN_7  = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "GPIO.DIR.PIN_7", signed=False, parent=self)
        self.PIN_8  = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "GPIO.DIR.PIN_8", signed=False, parent=self)
        self.PIN_9  = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "GPIO.DIR.PIN_9", signed=False, parent=self)
        self.PIN_10 = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "GPIO.DIR.PIN_10", signed=False, parent=self)
        self.PIN_11 = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "GPIO.DIR.PIN_11", signed=False, parent=self)
        self.PIN_12 = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "GPIO.DIR.PIN_12", signed=False, parent=self)
        self.PIN_13 = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "GPIO.DIR.PIN_13", signed=False, parent=self)
        self.PIN_14 = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "GPIO.DIR.PIN_14", signed=False, parent=self)
        self.PIN_15 = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "GPIO.DIR.PIN_15", signed=False, parent=self)
        self.PIN_16 = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "GPIO.DIR.PIN_16", signed=False, parent=self)
        self.PIN_17 = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "GPIO.DIR.PIN_17", signed=False, parent=self)
        self.PIN_18 = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "GPIO.DIR.PIN_18", signed=False, parent=self)

class _Gpio_IrqPolarity(Register):
    ADDRESS = 0x48002014
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PIN_0  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "GPIO.IRQ_POLARITY.PIN_0", signed=False, parent=self)
        self.PIN_1  = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "GPIO.IRQ_POLARITY.PIN_1", signed=False, parent=self)
        self.PIN_2  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "GPIO.IRQ_POLARITY.PIN_2", signed=False, parent=self)
        self.PIN_3  = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "GPIO.IRQ_POLARITY.PIN_3", signed=False, parent=self)
        self.PIN_4  = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "GPIO.IRQ_POLARITY.PIN_4", signed=False, parent=self)
        self.PIN_5  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "GPIO.IRQ_POLARITY.PIN_5", signed=False, parent=self)
        self.PIN_6  = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "GPIO.IRQ_POLARITY.PIN_6", signed=False, parent=self)
        self.PIN_7  = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "GPIO.IRQ_POLARITY.PIN_7", signed=False, parent=self)
        self.PIN_8  = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "GPIO.IRQ_POLARITY.PIN_8", signed=False, parent=self)
        self.PIN_9  = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "GPIO.IRQ_POLARITY.PIN_9", signed=False, parent=self)
        self.PIN_10 = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "GPIO.IRQ_POLARITY.PIN_10", signed=False, parent=self)
        self.PIN_11 = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "GPIO.IRQ_POLARITY.PIN_11", signed=False, parent=self)
        self.PIN_12 = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "GPIO.IRQ_POLARITY.PIN_12", signed=False, parent=self)
        self.PIN_13 = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "GPIO.IRQ_POLARITY.PIN_13", signed=False, parent=self)
        self.PIN_14 = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "GPIO.IRQ_POLARITY.PIN_14", signed=False, parent=self)
        self.PIN_15 = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "GPIO.IRQ_POLARITY.PIN_15", signed=False, parent=self)
        self.PIN_16 = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "GPIO.IRQ_POLARITY.PIN_16", signed=False, parent=self)
        self.PIN_17 = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "GPIO.IRQ_POLARITY.PIN_17", signed=False, parent=self)
        self.PIN_18 = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "GPIO.IRQ_POLARITY.PIN_18", signed=False, parent=self)

class _Gpio_IrqMask(Register):
    ADDRESS = 0x48002018
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PIN_0  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "GPIO.IRQ_MASK.PIN_0", signed=False, parent=self)
        self.PIN_1  = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "GPIO.IRQ_MASK.PIN_1", signed=False, parent=self)
        self.PIN_2  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "GPIO.IRQ_MASK.PIN_2", signed=False, parent=self)
        self.PIN_3  = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "GPIO.IRQ_MASK.PIN_3", signed=False, parent=self)
        self.PIN_4  = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "GPIO.IRQ_MASK.PIN_4", signed=False, parent=self)
        self.PIN_5  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "GPIO.IRQ_MASK.PIN_5", signed=False, parent=self)
        self.PIN_6  = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "GPIO.IRQ_MASK.PIN_6", signed=False, parent=self)
        self.PIN_7  = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "GPIO.IRQ_MASK.PIN_7", signed=False, parent=self)
        self.PIN_8  = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "GPIO.IRQ_MASK.PIN_8", signed=False, parent=self)
        self.PIN_9  = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "GPIO.IRQ_MASK.PIN_9", signed=False, parent=self)
        self.PIN_10 = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "GPIO.IRQ_MASK.PIN_10", signed=False, parent=self)
        self.PIN_11 = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "GPIO.IRQ_MASK.PIN_11", signed=False, parent=self)
        self.PIN_12 = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "GPIO.IRQ_MASK.PIN_12", signed=False, parent=self)
        self.PIN_13 = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "GPIO.IRQ_MASK.PIN_13", signed=False, parent=self)
        self.PIN_14 = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "GPIO.IRQ_MASK.PIN_14", signed=False, parent=self)
        self.PIN_15 = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "GPIO.IRQ_MASK.PIN_15", signed=False, parent=self)
        self.PIN_16 = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "GPIO.IRQ_MASK.PIN_16", signed=False, parent=self)
        self.PIN_17 = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "GPIO.IRQ_MASK.PIN_17", signed=False, parent=self)
        self.PIN_18 = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "GPIO.IRQ_MASK.PIN_18", signed=False, parent=self)

class _Gpio_IrqStatus(Register):
    ADDRESS = 0x4800201C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PIN_0  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "GPIO.IRQ_STATUS.PIN_0", signed=False, parent=self)
        self.PIN_1  = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "GPIO.IRQ_STATUS.PIN_1", signed=False, parent=self)
        self.PIN_2  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "GPIO.IRQ_STATUS.PIN_2", signed=False, parent=self)
        self.PIN_3  = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "GPIO.IRQ_STATUS.PIN_3", signed=False, parent=self)
        self.PIN_4  = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "GPIO.IRQ_STATUS.PIN_4", signed=False, parent=self)
        self.PIN_5  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "GPIO.IRQ_STATUS.PIN_5", signed=False, parent=self)
        self.PIN_6  = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "GPIO.IRQ_STATUS.PIN_6", signed=False, parent=self)
        self.PIN_7  = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "GPIO.IRQ_STATUS.PIN_7", signed=False, parent=self)
        self.PIN_8  = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "GPIO.IRQ_STATUS.PIN_8", signed=False, parent=self)
        self.PIN_9  = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "GPIO.IRQ_STATUS.PIN_9", signed=False, parent=self)
        self.PIN_10 = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "GPIO.IRQ_STATUS.PIN_10", signed=False, parent=self)
        self.PIN_11 = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "GPIO.IRQ_STATUS.PIN_11", signed=False, parent=self)
        self.PIN_12 = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "GPIO.IRQ_STATUS.PIN_12", signed=False, parent=self)
        self.PIN_13 = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "GPIO.IRQ_STATUS.PIN_13", signed=False, parent=self)
        self.PIN_14 = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "GPIO.IRQ_STATUS.PIN_14", signed=False, parent=self)
        self.PIN_15 = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "GPIO.IRQ_STATUS.PIN_15", signed=False, parent=self)
        self.PIN_16 = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "GPIO.IRQ_STATUS.PIN_16", signed=False, parent=self)
        self.PIN_17 = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "GPIO.IRQ_STATUS.PIN_17", signed=False, parent=self)
        self.PIN_18 = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "GPIO.IRQ_STATUS.PIN_18", signed=False, parent=self)

class _Gpio_IrqBothEdges(Register):
    ADDRESS = 0x48002020
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PIN_0  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "GPIO.IRQ_BOTH_EDGES.PIN_0", signed=False, parent=self)
        self.PIN_1  = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "GPIO.IRQ_BOTH_EDGES.PIN_1", signed=False, parent=self)
        self.PIN_2  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "GPIO.IRQ_BOTH_EDGES.PIN_2", signed=False, parent=self)
        self.PIN_3  = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "GPIO.IRQ_BOTH_EDGES.PIN_3", signed=False, parent=self)
        self.PIN_4  = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "GPIO.IRQ_BOTH_EDGES.PIN_4", signed=False, parent=self)
        self.PIN_5  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "GPIO.IRQ_BOTH_EDGES.PIN_5", signed=False, parent=self)
        self.PIN_6  = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "GPIO.IRQ_BOTH_EDGES.PIN_6", signed=False, parent=self)
        self.PIN_7  = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "GPIO.IRQ_BOTH_EDGES.PIN_7", signed=False, parent=self)
        self.PIN_8  = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "GPIO.IRQ_BOTH_EDGES.PIN_8", signed=False, parent=self)
        self.PIN_9  = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "GPIO.IRQ_BOTH_EDGES.PIN_9", signed=False, parent=self)
        self.PIN_10 = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "GPIO.IRQ_BOTH_EDGES.PIN_10", signed=False, parent=self)
        self.PIN_11 = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "GPIO.IRQ_BOTH_EDGES.PIN_11", signed=False, parent=self)
        self.PIN_12 = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "GPIO.IRQ_BOTH_EDGES.PIN_12", signed=False, parent=self)
        self.PIN_13 = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "GPIO.IRQ_BOTH_EDGES.PIN_13", signed=False, parent=self)
        self.PIN_14 = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "GPIO.IRQ_BOTH_EDGES.PIN_14", signed=False, parent=self)
        self.PIN_15 = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "GPIO.IRQ_BOTH_EDGES.PIN_15", signed=False, parent=self)
        self.PIN_16 = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "GPIO.IRQ_BOTH_EDGES.PIN_16", signed=False, parent=self)
        self.PIN_17 = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "GPIO.IRQ_BOTH_EDGES.PIN_17", signed=False, parent=self)
        self.PIN_18 = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "GPIO.IRQ_BOTH_EDGES.PIN_18", signed=False, parent=self)

class _Gpio_IrqEdgeLevel(Register):
    ADDRESS = 0x48002024
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PIN_0  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "GPIO.IRQ_EDGE_LEVEL.PIN_0", signed=False, parent=self)
        self.PIN_1  = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "GPIO.IRQ_EDGE_LEVEL.PIN_1", signed=False, parent=self)
        self.PIN_2  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "GPIO.IRQ_EDGE_LEVEL.PIN_2", signed=False, parent=self)
        self.PIN_3  = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "GPIO.IRQ_EDGE_LEVEL.PIN_3", signed=False, parent=self)
        self.PIN_4  = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "GPIO.IRQ_EDGE_LEVEL.PIN_4", signed=False, parent=self)
        self.PIN_5  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "GPIO.IRQ_EDGE_LEVEL.PIN_5", signed=False, parent=self)
        self.PIN_6  = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "GPIO.IRQ_EDGE_LEVEL.PIN_6", signed=False, parent=self)
        self.PIN_7  = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "GPIO.IRQ_EDGE_LEVEL.PIN_7", signed=False, parent=self)
        self.PIN_8  = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "GPIO.IRQ_EDGE_LEVEL.PIN_8", signed=False, parent=self)
        self.PIN_9  = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "GPIO.IRQ_EDGE_LEVEL.PIN_9", signed=False, parent=self)
        self.PIN_10 = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "GPIO.IRQ_EDGE_LEVEL.PIN_10", signed=False, parent=self)
        self.PIN_11 = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "GPIO.IRQ_EDGE_LEVEL.PIN_11", signed=False, parent=self)
        self.PIN_12 = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "GPIO.IRQ_EDGE_LEVEL.PIN_12", signed=False, parent=self)
        self.PIN_13 = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "GPIO.IRQ_EDGE_LEVEL.PIN_13", signed=False, parent=self)
        self.PIN_14 = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "GPIO.IRQ_EDGE_LEVEL.PIN_14", signed=False, parent=self)
        self.PIN_15 = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "GPIO.IRQ_EDGE_LEVEL.PIN_15", signed=False, parent=self)
        self.PIN_16 = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "GPIO.IRQ_EDGE_LEVEL.PIN_16", signed=False, parent=self)
        self.PIN_17 = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "GPIO.IRQ_EDGE_LEVEL.PIN_17", signed=False, parent=self)
        self.PIN_18 = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "GPIO.IRQ_EDGE_LEVEL.PIN_18", signed=False, parent=self)


class _Pic(AddressBlock):
    _NAME       = "PIC"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.IRQ_MASK_DATA   = _Pic_IrqMaskData(self)
        self.IRQ_MASK_SET    = _Pic_IrqMaskSet(self)
        self.IRQ_MASK_CLEAR  = _Pic_IrqMaskClear(self)
        self.IRQ_FLAG        = _Pic_IrqFlag(self)
        self.IRQ_SOURCE      = _Pic_IrqSource(self)
        self.IRQ_PENDM_DATA  = _Pic_IrqPendmData(self)
        self.IRQ_PENDM_SET   = _Pic_IrqPendmSet(self)
        self.IRQ_PENDM_CLEAR = _Pic_IrqPendmClear(self)
        self.IRQ_PEND_FLAG   = _Pic_IrqPendFlag(self)
        self.DS_SOURCE       = _Pic_DsSource(self)
        self.DS_RATIO        = _Pic_DsRatio(self)
        self.FAULT           = _Pic_Fault(self)
        self.FAULT_MASK      = _Pic_FaultMask(self)
        self.FAULT_PIN_MASK  = _Pic_FaultPinMask(self)


class _Pic_IrqMaskData(Register):
    ADDRESS = 0x48005000
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.UART         = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "PIC.IRQ_MASK_DATA.UART", signed=False, parent=self)
        self.TIMER_SYS    = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "PIC.IRQ_MASK_DATA.TIMER_SYS", signed=False, parent=self)
        self.SPI0         = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "PIC.IRQ_MASK_DATA.SPI0", signed=False, parent=self)
        self.I2C          = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "PIC.IRQ_MASK_DATA.I2C", signed=False, parent=self)
        self.MCC          = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "PIC.IRQ_MASK_DATA.MCC", signed=False, parent=self)
        self.PWM_C        = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "PIC.IRQ_MASK_DATA.PWM_C", signed=False, parent=self)
        self.PWM_Z        = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "PIC.IRQ_MASK_DATA.PWM_Z", signed=False, parent=self)
        self.ADC_I        = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "PIC.IRQ_MASK_DATA.ADC_I", signed=False, parent=self)
        self.ADC_U        = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "PIC.IRQ_MASK_DATA.ADC_U", signed=False, parent=self)
        self.SPI1         = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "PIC.IRQ_MASK_DATA.SPI1", signed=False, parent=self)
        self.TIMER_ADV    = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "PIC.IRQ_MASK_DATA.TIMER_ADV", signed=False, parent=self)
        self.TIMER0_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "PIC.IRQ_MASK_DATA.TIMER0_BASIC", signed=False, parent=self)
        self.TIMER1_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "PIC.IRQ_MASK_DATA.TIMER1_BASIC", signed=False, parent=self)
        self.TIMER2_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "PIC.IRQ_MASK_DATA.TIMER2_BASIC", signed=False, parent=self)
        self.FAULT        = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "PIC.IRQ_MASK_DATA.FAULT", signed=False, parent=self)
        self.GPIO0        = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "PIC.IRQ_MASK_DATA.GPIO0", signed=False, parent=self)
        self.GPIO1        = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "PIC.IRQ_MASK_DATA.GPIO1", signed=False, parent=self)
        self.GPIO2        = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "PIC.IRQ_MASK_DATA.GPIO2", signed=False, parent=self)
        self.GPIO3        = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "PIC.IRQ_MASK_DATA.GPIO3", signed=False, parent=self)
        self.GPIO4        = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "PIC.IRQ_MASK_DATA.GPIO4", signed=False, parent=self)
        self.GPIO5        = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "PIC.IRQ_MASK_DATA.GPIO5", signed=False, parent=self)
        self.GPIO6        = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "PIC.IRQ_MASK_DATA.GPIO6", signed=False, parent=self)
        self.GPIO7        = Field(self.ADDRESS, self.WIDTH, 0x02000000, 25, "PIC.IRQ_MASK_DATA.GPIO7", signed=False, parent=self)
        self.GPIO8        = Field(self.ADDRESS, self.WIDTH, 0x04000000, 26, "PIC.IRQ_MASK_DATA.GPIO8", signed=False, parent=self)
        self.GPIO9        = Field(self.ADDRESS, self.WIDTH, 0x08000000, 27, "PIC.IRQ_MASK_DATA.GPIO9", signed=False, parent=self)

class _Pic_IrqMaskSet(Register):
    ADDRESS = 0x48005004
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.UART         = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "PIC.IRQ_MASK_SET.UART", signed=False, parent=self)
        self.TIMER_SYS    = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "PIC.IRQ_MASK_SET.TIMER_SYS", signed=False, parent=self)
        self.SPI0         = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "PIC.IRQ_MASK_SET.SPI0", signed=False, parent=self)
        self.I2C          = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "PIC.IRQ_MASK_SET.I2C", signed=False, parent=self)
        self.MCC          = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "PIC.IRQ_MASK_SET.MCC", signed=False, parent=self)
        self.PWM_C        = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "PIC.IRQ_MASK_SET.PWM_C", signed=False, parent=self)
        self.PWM_Z        = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "PIC.IRQ_MASK_SET.PWM_Z", signed=False, parent=self)
        self.ADC_I        = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "PIC.IRQ_MASK_SET.ADC_I", signed=False, parent=self)
        self.ADC_U        = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "PIC.IRQ_MASK_SET.ADC_U", signed=False, parent=self)
        self.SPI1         = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "PIC.IRQ_MASK_SET.SPI1", signed=False, parent=self)
        self.TIMER_ADV    = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "PIC.IRQ_MASK_SET.TIMER_ADV", signed=False, parent=self)
        self.TIMER0_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "PIC.IRQ_MASK_SET.TIMER0_BASIC", signed=False, parent=self)
        self.TIMER1_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "PIC.IRQ_MASK_SET.TIMER1_BASIC", signed=False, parent=self)
        self.TIMER2_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "PIC.IRQ_MASK_SET.TIMER2_BASIC", signed=False, parent=self)
        self.FAULT        = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "PIC.IRQ_MASK_SET.FAULT", signed=False, parent=self)
        self.GPIO0        = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "PIC.IRQ_MASK_SET.GPIO0", signed=False, parent=self)
        self.GPIO1        = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "PIC.IRQ_MASK_SET.GPIO1", signed=False, parent=self)
        self.GPIO2        = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "PIC.IRQ_MASK_SET.GPIO2", signed=False, parent=self)
        self.GPIO3        = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "PIC.IRQ_MASK_SET.GPIO3", signed=False, parent=self)
        self.GPIO4        = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "PIC.IRQ_MASK_SET.GPIO4", signed=False, parent=self)
        self.GPIO5        = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "PIC.IRQ_MASK_SET.GPIO5", signed=False, parent=self)
        self.GPIO6        = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "PIC.IRQ_MASK_SET.GPIO6", signed=False, parent=self)
        self.GPIO7        = Field(self.ADDRESS, self.WIDTH, 0x02000000, 25, "PIC.IRQ_MASK_SET.GPIO7", signed=False, parent=self)
        self.GPIO8        = Field(self.ADDRESS, self.WIDTH, 0x04000000, 26, "PIC.IRQ_MASK_SET.GPIO8", signed=False, parent=self)
        self.GPIO9        = Field(self.ADDRESS, self.WIDTH, 0x08000000, 27, "PIC.IRQ_MASK_SET.GPIO9", signed=False, parent=self)

class _Pic_IrqMaskClear(Register):
    ADDRESS = 0x48005008
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.UART         = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "PIC.IRQ_MASK_CLEAR.UART", signed=False, parent=self)
        self.TIMER_SYS    = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "PIC.IRQ_MASK_CLEAR.TIMER_SYS", signed=False, parent=self)
        self.SPI0         = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "PIC.IRQ_MASK_CLEAR.SPI0", signed=False, parent=self)
        self.I2C          = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "PIC.IRQ_MASK_CLEAR.I2C", signed=False, parent=self)
        self.MCC          = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "PIC.IRQ_MASK_CLEAR.MCC", signed=False, parent=self)
        self.PWM_C        = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "PIC.IRQ_MASK_CLEAR.PWM_C", signed=False, parent=self)
        self.PWM_Z        = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "PIC.IRQ_MASK_CLEAR.PWM_Z", signed=False, parent=self)
        self.ADC_I        = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "PIC.IRQ_MASK_CLEAR.ADC_I", signed=False, parent=self)
        self.ADC_U        = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "PIC.IRQ_MASK_CLEAR.ADC_U", signed=False, parent=self)
        self.SPI1         = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "PIC.IRQ_MASK_CLEAR.SPI1", signed=False, parent=self)
        self.TIMER_ADV    = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "PIC.IRQ_MASK_CLEAR.TIMER_ADV", signed=False, parent=self)
        self.TIMER0_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "PIC.IRQ_MASK_CLEAR.TIMER0_BASIC", signed=False, parent=self)
        self.TIMER1_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "PIC.IRQ_MASK_CLEAR.TIMER1_BASIC", signed=False, parent=self)
        self.TIMER2_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "PIC.IRQ_MASK_CLEAR.TIMER2_BASIC", signed=False, parent=self)
        self.FAULT        = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "PIC.IRQ_MASK_CLEAR.FAULT", signed=False, parent=self)
        self.GPIO0        = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "PIC.IRQ_MASK_CLEAR.GPIO0", signed=False, parent=self)
        self.GPIO1        = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "PIC.IRQ_MASK_CLEAR.GPIO1", signed=False, parent=self)
        self.GPIO2        = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "PIC.IRQ_MASK_CLEAR.GPIO2", signed=False, parent=self)
        self.GPIO3        = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "PIC.IRQ_MASK_CLEAR.GPIO3", signed=False, parent=self)
        self.GPIO4        = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "PIC.IRQ_MASK_CLEAR.GPIO4", signed=False, parent=self)
        self.GPIO5        = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "PIC.IRQ_MASK_CLEAR.GPIO5", signed=False, parent=self)
        self.GPIO6        = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "PIC.IRQ_MASK_CLEAR.GPIO6", signed=False, parent=self)
        self.GPIO7        = Field(self.ADDRESS, self.WIDTH, 0x02000000, 25, "PIC.IRQ_MASK_CLEAR.GPIO7", signed=False, parent=self)
        self.GPIO8        = Field(self.ADDRESS, self.WIDTH, 0x04000000, 26, "PIC.IRQ_MASK_CLEAR.GPIO8", signed=False, parent=self)
        self.GPIO9        = Field(self.ADDRESS, self.WIDTH, 0x08000000, 27, "PIC.IRQ_MASK_CLEAR.GPIO9", signed=False, parent=self)

class _Pic_IrqFlag(Register):
    ADDRESS = 0x4800500C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.UART         = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "PIC.IRQ_FLAG.UART", signed=False, parent=self)
        self.TIMER_SYS    = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "PIC.IRQ_FLAG.TIMER_SYS", signed=False, parent=self)
        self.SPI0         = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "PIC.IRQ_FLAG.SPI0", signed=False, parent=self)
        self.I2C          = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "PIC.IRQ_FLAG.I2C", signed=False, parent=self)
        self.MCC          = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "PIC.IRQ_FLAG.MCC", signed=False, parent=self)
        self.PWM_C        = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "PIC.IRQ_FLAG.PWM_C", signed=False, parent=self)
        self.PWM_Z        = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "PIC.IRQ_FLAG.PWM_Z", signed=False, parent=self)
        self.ADC_I        = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "PIC.IRQ_FLAG.ADC_I", signed=False, parent=self)
        self.ADC_U        = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "PIC.IRQ_FLAG.ADC_U", signed=False, parent=self)
        self.SPI1         = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "PIC.IRQ_FLAG.SPI1", signed=False, parent=self)
        self.TIMER_ADV    = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "PIC.IRQ_FLAG.TIMER_ADV", signed=False, parent=self)
        self.TIMER0_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "PIC.IRQ_FLAG.TIMER0_BASIC", signed=False, parent=self)
        self.TIMER1_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "PIC.IRQ_FLAG.TIMER1_BASIC", signed=False, parent=self)
        self.TIMER2_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "PIC.IRQ_FLAG.TIMER2_BASIC", signed=False, parent=self)
        self.FAULT        = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "PIC.IRQ_FLAG.FAULT", signed=False, parent=self)
        self.GPIO0        = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "PIC.IRQ_FLAG.GPIO0", signed=False, parent=self)
        self.GPIO1        = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "PIC.IRQ_FLAG.GPIO1", signed=False, parent=self)
        self.GPIO2        = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "PIC.IRQ_FLAG.GPIO2", signed=False, parent=self)
        self.GPIO3        = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "PIC.IRQ_FLAG.GPIO3", signed=False, parent=self)
        self.GPIO4        = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "PIC.IRQ_FLAG.GPIO4", signed=False, parent=self)
        self.GPIO5        = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "PIC.IRQ_FLAG.GPIO5", signed=False, parent=self)
        self.GPIO6        = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "PIC.IRQ_FLAG.GPIO6", signed=False, parent=self)
        self.GPIO7        = Field(self.ADDRESS, self.WIDTH, 0x02000000, 25, "PIC.IRQ_FLAG.GPIO7", signed=False, parent=self)
        self.GPIO8        = Field(self.ADDRESS, self.WIDTH, 0x04000000, 26, "PIC.IRQ_FLAG.GPIO8", signed=False, parent=self)
        self.GPIO9        = Field(self.ADDRESS, self.WIDTH, 0x08000000, 27, "PIC.IRQ_FLAG.GPIO9", signed=False, parent=self)

class _Pic_IrqSource(Register):
    ADDRESS = 0x48005010
    WIDTH = 4
    DEFAULT = 0x000001FF

    def __init__(self, parent):
        self._PARENT = parent

        self.IRQ_SOURCE = Field(self.ADDRESS, self.WIDTH, 0x000001FF,  0, "PIC.IRQ_SOURCE.IRQ_SOURCE", signed=False, parent=self)

class _Pic_IrqPendmData(Register):
    ADDRESS = 0x48005014
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.UART         = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "PIC.IRQ_PENDM_DATA.UART", signed=False, parent=self)
        self.TIMER_SYS    = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "PIC.IRQ_PENDM_DATA.TIMER_SYS", signed=False, parent=self)
        self.SPI0         = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "PIC.IRQ_PENDM_DATA.SPI0", signed=False, parent=self)
        self.I2C          = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "PIC.IRQ_PENDM_DATA.I2C", signed=False, parent=self)
        self.MCC          = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "PIC.IRQ_PENDM_DATA.MCC", signed=False, parent=self)
        self.PWM_C        = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "PIC.IRQ_PENDM_DATA.PWM_C", signed=False, parent=self)
        self.PWM_Z        = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "PIC.IRQ_PENDM_DATA.PWM_Z", signed=False, parent=self)
        self.ADC_I        = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "PIC.IRQ_PENDM_DATA.ADC_I", signed=False, parent=self)
        self.ADC_U        = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "PIC.IRQ_PENDM_DATA.ADC_U", signed=False, parent=self)
        self.SPI1         = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "PIC.IRQ_PENDM_DATA.SPI1", signed=False, parent=self)
        self.TIMER_ADV    = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "PIC.IRQ_PENDM_DATA.TIMER_ADV", signed=False, parent=self)
        self.TIMER0_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "PIC.IRQ_PENDM_DATA.TIMER0_BASIC", signed=False, parent=self)
        self.TIMER1_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "PIC.IRQ_PENDM_DATA.TIMER1_BASIC", signed=False, parent=self)
        self.TIMER2_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "PIC.IRQ_PENDM_DATA.TIMER2_BASIC", signed=False, parent=self)
        self.FAULT        = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "PIC.IRQ_PENDM_DATA.FAULT", signed=False, parent=self)
        self.GPIO0        = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "PIC.IRQ_PENDM_DATA.GPIO0", signed=False, parent=self)
        self.GPIO1        = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "PIC.IRQ_PENDM_DATA.GPIO1", signed=False, parent=self)
        self.GPIO2        = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "PIC.IRQ_PENDM_DATA.GPIO2", signed=False, parent=self)
        self.GPIO3        = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "PIC.IRQ_PENDM_DATA.GPIO3", signed=False, parent=self)
        self.GPIO4        = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "PIC.IRQ_PENDM_DATA.GPIO4", signed=False, parent=self)
        self.GPIO5        = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "PIC.IRQ_PENDM_DATA.GPIO5", signed=False, parent=self)
        self.GPIO6        = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "PIC.IRQ_PENDM_DATA.GPIO6", signed=False, parent=self)
        self.GPIO7        = Field(self.ADDRESS, self.WIDTH, 0x02000000, 25, "PIC.IRQ_PENDM_DATA.GPIO7", signed=False, parent=self)
        self.GPIO8        = Field(self.ADDRESS, self.WIDTH, 0x04000000, 26, "PIC.IRQ_PENDM_DATA.GPIO8", signed=False, parent=self)
        self.GPIO9        = Field(self.ADDRESS, self.WIDTH, 0x08000000, 27, "PIC.IRQ_PENDM_DATA.GPIO9", signed=False, parent=self)

class _Pic_IrqPendmSet(Register):
    ADDRESS = 0x48005018
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.UART         = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "PIC.IRQ_PENDM_SET.UART", signed=False, parent=self)
        self.TIMER_SYS    = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "PIC.IRQ_PENDM_SET.TIMER_SYS", signed=False, parent=self)
        self.SPI0         = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "PIC.IRQ_PENDM_SET.SPI0", signed=False, parent=self)
        self.I2C          = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "PIC.IRQ_PENDM_SET.I2C", signed=False, parent=self)
        self.MCC          = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "PIC.IRQ_PENDM_SET.MCC", signed=False, parent=self)
        self.PWM_C        = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "PIC.IRQ_PENDM_SET.PWM_C", signed=False, parent=self)
        self.PWM_Z        = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "PIC.IRQ_PENDM_SET.PWM_Z", signed=False, parent=self)
        self.ADC_I        = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "PIC.IRQ_PENDM_SET.ADC_I", signed=False, parent=self)
        self.ADC_U        = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "PIC.IRQ_PENDM_SET.ADC_U", signed=False, parent=self)
        self.SPI1         = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "PIC.IRQ_PENDM_SET.SPI1", signed=False, parent=self)
        self.TIMER_ADV    = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "PIC.IRQ_PENDM_SET.TIMER_ADV", signed=False, parent=self)
        self.TIMER0_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "PIC.IRQ_PENDM_SET.TIMER0_BASIC", signed=False, parent=self)
        self.TIMER1_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "PIC.IRQ_PENDM_SET.TIMER1_BASIC", signed=False, parent=self)
        self.TIMER2_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "PIC.IRQ_PENDM_SET.TIMER2_BASIC", signed=False, parent=self)
        self.FAULT        = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "PIC.IRQ_PENDM_SET.FAULT", signed=False, parent=self)
        self.GPIO0        = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "PIC.IRQ_PENDM_SET.GPIO0", signed=False, parent=self)
        self.GPIO1        = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "PIC.IRQ_PENDM_SET.GPIO1", signed=False, parent=self)
        self.GPIO2        = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "PIC.IRQ_PENDM_SET.GPIO2", signed=False, parent=self)
        self.GPIO3        = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "PIC.IRQ_PENDM_SET.GPIO3", signed=False, parent=self)
        self.GPIO4        = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "PIC.IRQ_PENDM_SET.GPIO4", signed=False, parent=self)
        self.GPIO5        = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "PIC.IRQ_PENDM_SET.GPIO5", signed=False, parent=self)
        self.GPIO6        = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "PIC.IRQ_PENDM_SET.GPIO6", signed=False, parent=self)
        self.GPIO7        = Field(self.ADDRESS, self.WIDTH, 0x02000000, 25, "PIC.IRQ_PENDM_SET.GPIO7", signed=False, parent=self)
        self.GPIO8        = Field(self.ADDRESS, self.WIDTH, 0x04000000, 26, "PIC.IRQ_PENDM_SET.GPIO8", signed=False, parent=self)
        self.GPIO9        = Field(self.ADDRESS, self.WIDTH, 0x08000000, 27, "PIC.IRQ_PENDM_SET.GPIO9", signed=False, parent=self)

class _Pic_IrqPendmClear(Register):
    ADDRESS = 0x4800501C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.UART         = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "PIC.IRQ_PENDM_CLEAR.UART", signed=False, parent=self)
        self.TIMER_SYS    = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "PIC.IRQ_PENDM_CLEAR.TIMER_SYS", signed=False, parent=self)
        self.SPI0         = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "PIC.IRQ_PENDM_CLEAR.SPI0", signed=False, parent=self)
        self.I2C          = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "PIC.IRQ_PENDM_CLEAR.I2C", signed=False, parent=self)
        self.MCC          = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "PIC.IRQ_PENDM_CLEAR.MCC", signed=False, parent=self)
        self.PWM_C        = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "PIC.IRQ_PENDM_CLEAR.PWM_C", signed=False, parent=self)
        self.PWM_Z        = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "PIC.IRQ_PENDM_CLEAR.PWM_Z", signed=False, parent=self)
        self.ADC_I        = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "PIC.IRQ_PENDM_CLEAR.ADC_I", signed=False, parent=self)
        self.ADC_U        = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "PIC.IRQ_PENDM_CLEAR.ADC_U", signed=False, parent=self)
        self.SPI1         = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "PIC.IRQ_PENDM_CLEAR.SPI1", signed=False, parent=self)
        self.TIMER_ADV    = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "PIC.IRQ_PENDM_CLEAR.TIMER_ADV", signed=False, parent=self)
        self.TIMER0_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "PIC.IRQ_PENDM_CLEAR.TIMER0_BASIC", signed=False, parent=self)
        self.TIMER1_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "PIC.IRQ_PENDM_CLEAR.TIMER1_BASIC", signed=False, parent=self)
        self.TIMER2_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "PIC.IRQ_PENDM_CLEAR.TIMER2_BASIC", signed=False, parent=self)
        self.FAULT        = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "PIC.IRQ_PENDM_CLEAR.FAULT", signed=False, parent=self)
        self.GPIO0        = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "PIC.IRQ_PENDM_CLEAR.GPIO0", signed=False, parent=self)
        self.GPIO1        = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "PIC.IRQ_PENDM_CLEAR.GPIO1", signed=False, parent=self)
        self.GPIO2        = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "PIC.IRQ_PENDM_CLEAR.GPIO2", signed=False, parent=self)
        self.GPIO3        = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "PIC.IRQ_PENDM_CLEAR.GPIO3", signed=False, parent=self)
        self.GPIO4        = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "PIC.IRQ_PENDM_CLEAR.GPIO4", signed=False, parent=self)
        self.GPIO5        = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "PIC.IRQ_PENDM_CLEAR.GPIO5", signed=False, parent=self)
        self.GPIO6        = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "PIC.IRQ_PENDM_CLEAR.GPIO6", signed=False, parent=self)
        self.GPIO7        = Field(self.ADDRESS, self.WIDTH, 0x02000000, 25, "PIC.IRQ_PENDM_CLEAR.GPIO7", signed=False, parent=self)
        self.GPIO8        = Field(self.ADDRESS, self.WIDTH, 0x04000000, 26, "PIC.IRQ_PENDM_CLEAR.GPIO8", signed=False, parent=self)
        self.GPIO9        = Field(self.ADDRESS, self.WIDTH, 0x08000000, 27, "PIC.IRQ_PENDM_CLEAR.GPIO9", signed=False, parent=self)

class _Pic_IrqPendFlag(Register):
    ADDRESS = 0x48005020
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.UART         = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "PIC.IRQ_PEND_FLAG.UART", signed=False, parent=self)
        self.TIMER_SYS    = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "PIC.IRQ_PEND_FLAG.TIMER_SYS", signed=False, parent=self)
        self.SPI0         = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "PIC.IRQ_PEND_FLAG.SPI0", signed=False, parent=self)
        self.I2C          = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "PIC.IRQ_PEND_FLAG.I2C", signed=False, parent=self)
        self.MCC          = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "PIC.IRQ_PEND_FLAG.MCC", signed=False, parent=self)
        self.PWM_C        = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "PIC.IRQ_PEND_FLAG.PWM_C", signed=False, parent=self)
        self.PWM_Z        = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "PIC.IRQ_PEND_FLAG.PWM_Z", signed=False, parent=self)
        self.ADC_I        = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "PIC.IRQ_PEND_FLAG.ADC_I", signed=False, parent=self)
        self.ADC_U        = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "PIC.IRQ_PEND_FLAG.ADC_U", signed=False, parent=self)
        self.SPI1         = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "PIC.IRQ_PEND_FLAG.SPI1", signed=False, parent=self)
        self.TIMER_ADV    = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "PIC.IRQ_PEND_FLAG.TIMER_ADV", signed=False, parent=self)
        self.TIMER0_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "PIC.IRQ_PEND_FLAG.TIMER0_BASIC", signed=False, parent=self)
        self.TIMER1_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "PIC.IRQ_PEND_FLAG.TIMER1_BASIC", signed=False, parent=self)
        self.TIMER2_BASIC = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "PIC.IRQ_PEND_FLAG.TIMER2_BASIC", signed=False, parent=self)
        self.FAULT        = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "PIC.IRQ_PEND_FLAG.FAULT", signed=False, parent=self)
        self.GPIO0        = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "PIC.IRQ_PEND_FLAG.GPIO0", signed=False, parent=self)
        self.GPIO1        = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "PIC.IRQ_PEND_FLAG.GPIO1", signed=False, parent=self)
        self.GPIO2        = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "PIC.IRQ_PEND_FLAG.GPIO2", signed=False, parent=self)
        self.GPIO3        = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "PIC.IRQ_PEND_FLAG.GPIO3", signed=False, parent=self)
        self.GPIO4        = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "PIC.IRQ_PEND_FLAG.GPIO4", signed=False, parent=self)
        self.GPIO5        = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "PIC.IRQ_PEND_FLAG.GPIO5", signed=False, parent=self)
        self.GPIO6        = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "PIC.IRQ_PEND_FLAG.GPIO6", signed=False, parent=self)
        self.GPIO7        = Field(self.ADDRESS, self.WIDTH, 0x02000000, 25, "PIC.IRQ_PEND_FLAG.GPIO7", signed=False, parent=self)
        self.GPIO8        = Field(self.ADDRESS, self.WIDTH, 0x04000000, 26, "PIC.IRQ_PEND_FLAG.GPIO8", signed=False, parent=self)
        self.GPIO9        = Field(self.ADDRESS, self.WIDTH, 0x08000000, 27, "PIC.IRQ_PEND_FLAG.GPIO9", signed=False, parent=self)

class _Pic_DsSource(Register):
    ADDRESS = 0x48005024
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.DS_SOURCE = Field(self.ADDRESS, self.WIDTH, 0x0000001F,  0, "PIC.DS_SOURCE.DS_SOURCE", signed=False, parent=self)

class _Pic_DsRatio(Register):
    ADDRESS = 0x48005028
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.DS_RATIO = Field(self.ADDRESS, self.WIDTH, 0x000000FF,  0, "PIC.DS_RATIO.DS_RATIO", signed=False, parent=self)

class _Pic_Fault(Register):
    ADDRESS = 0x4800502C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.GDRV_FLT             = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "PIC.FAULT.GDRV_FLT", signed=False, parent=self)
        self.GDRV_BST_UVLO_FLT    = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "PIC.FAULT.GDRV_BST_UVLO_FLT", signed=False, parent=self)
        self.VS_UVLO_FLT          = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "PIC.FAULT.VS_UVLO_FLT", signed=False, parent=self)
        self.VDRV_UVLO_FLT        = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "PIC.FAULT.VDRV_UVLO_FLT", signed=False, parent=self)
        self.VDRV_UVLO_WARN_FLT   = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "PIC.FAULT.VDRV_UVLO_WARN_FLT", signed=False, parent=self)
        self.GDRV_VGS_FLT         = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "PIC.FAULT.GDRV_VGS_FLT", signed=False, parent=self)
        self.GDRV_OCP_FLT         = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "PIC.FAULT.GDRV_OCP_FLT", signed=False, parent=self)
        self.ADC_OV2V_FLAG        = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "PIC.FAULT.ADC_OV2V_FLAG", signed=False, parent=self)
        self.ADC_WTCHDG_FAIL_FLAG = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "PIC.FAULT.ADC_WTCHDG_FAIL_FLAG", signed=False, parent=self)
        self.ADC_FLT              = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "PIC.FAULT.ADC_FLT", signed=False, parent=self)
        self.IRQ_PMU_FLT          = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "PIC.FAULT.IRQ_PMU_FLT", signed=False, parent=self)

class _Pic_FaultMask(Register):
    ADDRESS = 0x48005030
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.GDRV_FLT_MASK             = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "PIC.FAULT_MASK.GDRV_FLT_MASK", signed=False, parent=self)
        self.GDRV_BST_UVLO_FLT_MASK    = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "PIC.FAULT_MASK.GDRV_BST_UVLO_FLT_MASK", signed=False, parent=self)
        self.VS_UVLO_FLT_MASK          = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "PIC.FAULT_MASK.VS_UVLO_FLT_MASK", signed=False, parent=self)
        self.VDRV_UVLO_FLT_MASK        = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "PIC.FAULT_MASK.VDRV_UVLO_FLT_MASK", signed=False, parent=self)
        self.VDRV_UVLO_WARN_FLT_MASK   = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "PIC.FAULT_MASK.VDRV_UVLO_WARN_FLT_MASK", signed=False, parent=self)
        self.GDRV_VGS_FLT_MASK         = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "PIC.FAULT_MASK.GDRV_VGS_FLT_MASK", signed=False, parent=self)
        self.GDRV_OCP_FLT_MASK         = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "PIC.FAULT_MASK.GDRV_OCP_FLT_MASK", signed=False, parent=self)
        self.ADC_OV2V_FLAG_MASK        = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "PIC.FAULT_MASK.ADC_OV2V_FLAG_MASK", signed=False, parent=self)
        self.ADC_WTCHDG_FAIL_FLAG_MASK = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "PIC.FAULT_MASK.ADC_WTCHDG_FAIL_FLAG_MASK", signed=False, parent=self)
        self.ADC_FLT_MASK              = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "PIC.FAULT_MASK.ADC_FLT_MASK", signed=False, parent=self)
        self.IRQ_PMU_FLT_MASK          = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "PIC.FAULT_MASK.IRQ_PMU_FLT_MASK", signed=False, parent=self)

class _Pic_FaultPinMask(Register):
    ADDRESS = 0x48005034
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.GDRV_FLT_PIN_MASK             = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "PIC.FAULT_PIN_MASK.GDRV_FLT_PIN_MASK", signed=False, parent=self)
        self.GDRV_BST_UVLO_FLT_PIN_MASK    = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "PIC.FAULT_PIN_MASK.GDRV_BST_UVLO_FLT_PIN_MASK", signed=False, parent=self)
        self.VS_UVLO_FLT_PIN_MASK          = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "PIC.FAULT_PIN_MASK.VS_UVLO_FLT_PIN_MASK", signed=False, parent=self)
        self.VDRV_UVLO_FLT_PIN_MASK        = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "PIC.FAULT_PIN_MASK.VDRV_UVLO_FLT_PIN_MASK", signed=False, parent=self)
        self.VDRV_UVLO_WARN_FLT_PIN_MASK   = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "PIC.FAULT_PIN_MASK.VDRV_UVLO_WARN_FLT_PIN_MASK", signed=False, parent=self)
        self.GDRV_VGS_FLT_PIN_MASK         = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "PIC.FAULT_PIN_MASK.GDRV_VGS_FLT_PIN_MASK", signed=False, parent=self)
        self.GDRV_OCP_FLT_PIN_MASK         = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "PIC.FAULT_PIN_MASK.GDRV_OCP_FLT_PIN_MASK", signed=False, parent=self)
        self.ADC_OV2V_FLAG_PIN_MASK        = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "PIC.FAULT_PIN_MASK.ADC_OV2V_FLAG_PIN_MASK", signed=False, parent=self)
        self.ADC_WTCHDG_FAIL_FLAG_PIN_MASK = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "PIC.FAULT_PIN_MASK.ADC_WTCHDG_FAIL_FLAG_PIN_MASK", signed=False, parent=self)
        self.ADC_FLT_PIN_MASK              = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "PIC.FAULT_PIN_MASK.ADC_FLT_PIN_MASK", signed=False, parent=self)


class _Spi0(AddressBlock):
    _NAME       = "SPI0"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.STATUS           = _Spi0_Status(self)
        self.GEN_CONFIG       = _Spi0_GenConfig(self)
        self.SCLK_CONFIG0     = _Spi0_SclkConfig0(self)
        self.SYNC_CONFIG0     = _Spi0_SyncConfig0(self)
        self.TRANSFER_CONFIG0 = _Spi0_TransferConfig0(self)
        self.SCLK_CONFIG1     = _Spi0_SclkConfig1(self)
        self.SYNC_CONFIG1     = _Spi0_SyncConfig1(self)
        self.TRANSFER_CONFIG1 = _Spi0_TransferConfig1(self)
        self.SCLK_CONFIG2     = _Spi0_SclkConfig2(self)
        self.SYNC_CONFIG2     = _Spi0_SyncConfig2(self)
        self.TRANSFER_CONFIG2 = _Spi0_TransferConfig2(self)
        self.RD_BUFFER0       = _Spi0_RdBuffer0(self)
        self.RD_BUFFER1       = _Spi0_RdBuffer1(self)
        self.RD_BUFFER2       = _Spi0_RdBuffer2(self)
        self.RD_BUFFER3       = _Spi0_RdBuffer3(self)
        self.WR_BUFFER0       = _Spi0_WrBuffer0(self)
        self.WR_BUFFER1       = _Spi0_WrBuffer1(self)
        self.WR_BUFFER2       = _Spi0_WrBuffer2(self)
        self.WR_BUFFER3       = _Spi0_WrBuffer3(self)


class _Spi0_Status(Register):
    ADDRESS = 0x48009000
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.TRANSACT_DONE                  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "SPI0.STATUS.TRANSACT_DONE", signed=False, parent=self)
        self.TRANSACT_STARTED               = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "SPI0.STATUS.TRANSACT_STARTED", signed=False, parent=self)
        self.TRANSACT_DONE_SINCE_STATUS_CLR = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "SPI0.STATUS.TRANSACT_DONE_SINCE_STATUS_CLR", signed=False, parent=self)
        self.BUSY1                          = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "SPI0.STATUS.BUSY1", signed=False, parent=self)
        self.BUSY2                          = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "SPI0.STATUS.BUSY2", signed=False, parent=self)
        self.CONT_ON                        = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "SPI0.STATUS.CONT_ON", signed=False, parent=self)
        self.RX_CORK                        = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "SPI0.STATUS.RX_CORK", signed=False, parent=self)

class _Spi0_GenConfig(Register):
    ADDRESS = 0x48009004
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.ENABLE           = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "SPI0.GEN_CONFIG.ENABLE", signed=False, parent=self)
        self.CONFIG_SEL       = Field(self.ADDRESS, self.WIDTH, 0x00000006,  1, "SPI0.GEN_CONFIG.CONFIG_SEL", signed=False, parent=self)
        self.START_NSTOP      = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "SPI0.GEN_CONFIG.START_NSTOP", signed=False, parent=self)
        self.START_PULSE      = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "SPI0.GEN_CONFIG.START_PULSE", signed=False, parent=self)
        self.READ_REQUEST     = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "SPI0.GEN_CONFIG.READ_REQUEST", signed=False, parent=self)
        self.READ_REQUEST_CLR = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "SPI0.GEN_CONFIG.READ_REQUEST_CLR", signed=False, parent=self)

class _Spi0_SclkConfig0(Register):
    ADDRESS = 0x48009008
    WIDTH = 4
    DEFAULT = 0x00000010

    def __init__(self, parent):
        self._PARENT = parent

        self.CLK_DIV       = Field(self.ADDRESS, self.WIDTH, 0x000000FF,  0, "SPI0.SCLK_CONFIG0.CLK_DIV", signed=False, parent=self)
        self.CS_SETTLE_TIM = Field(self.ADDRESS, self.WIDTH, 0x0000FF00,  8, "SPI0.SCLK_CONFIG0.CS_SETTLE_TIM", signed=False, parent=self)
        self.PAUSE_TIM     = Field(self.ADDRESS, self.WIDTH, 0x00FF0000, 16, "SPI0.SCLK_CONFIG0.PAUSE_TIM", signed=False, parent=self)

class _Spi0_SyncConfig0(Register):
    ADDRESS = 0x4800900C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.STROBE_MASK      = Field(self.ADDRESS, self.WIDTH, 0x000001FF,  0, "SPI0.SYNC_CONFIG0.STROBE_MASK", signed=False, parent=self)
        self.SW_STROBE_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "SPI0.SYNC_CONFIG0.SW_STROBE_ENABLE", signed=False, parent=self)
        self.HW_STROBE_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "SPI0.SYNC_CONFIG0.HW_STROBE_ENABLE", signed=False, parent=self)
        self.SEC_CYCLE_ACTIVE = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "SPI0.SYNC_CONFIG0.SEC_CYCLE_ACTIVE", signed=False, parent=self)

class _Spi0_TransferConfig0(Register):
    ADDRESS = 0x48009010
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.SCLK_PHASE      = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "SPI0.TRANSFER_CONFIG0.SCLK_PHASE", signed=False, parent=self)
        self.SCLK_POL        = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "SPI0.TRANSFER_CONFIG0.SCLK_POL", signed=False, parent=self)
        self.CS_POL          = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "SPI0.TRANSFER_CONFIG0.CS_POL", signed=False, parent=self)
        self.CS0_ENABLE      = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "SPI0.TRANSFER_CONFIG0.CS0_ENABLE", signed=False, parent=self)
        self.CS1_ENABLE      = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "SPI0.TRANSFER_CONFIG0.CS1_ENABLE", signed=False, parent=self)
        self.CS2_ENABLE      = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "SPI0.TRANSFER_CONFIG0.CS2_ENABLE", signed=False, parent=self)
        self.APPLY_TAD       = Field(self.ADDRESS, self.WIDTH, 0x000000C0,  6, "SPI0.TRANSFER_CONFIG0.APPLY_TAD", signed=False, parent=self)
        self.DATA_SIZE_SEL_0 = Field(self.ADDRESS, self.WIDTH, 0x00000F00,  8, "SPI0.TRANSFER_CONFIG0.DATA_SIZE_SEL_0", signed=False, parent=self)
        self.DATA_SIZE_SEL_1 = Field(self.ADDRESS, self.WIDTH, 0x0000F000, 12, "SPI0.TRANSFER_CONFIG0.DATA_SIZE_SEL_1", signed=False, parent=self)

class _Spi0_SclkConfig1(Register):
    ADDRESS = 0x48009014
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.CLK_DIV       = Field(self.ADDRESS, self.WIDTH, 0x000000FE,  1, "SPI0.SCLK_CONFIG1.CLK_DIV", signed=False, parent=self)
        self.CS_SETTLE_TIM = Field(self.ADDRESS, self.WIDTH, 0x0000FF00,  8, "SPI0.SCLK_CONFIG1.CS_SETTLE_TIM", signed=False, parent=self)
        self.PAUSE_TIM     = Field(self.ADDRESS, self.WIDTH, 0x00FF0000, 16, "SPI0.SCLK_CONFIG1.PAUSE_TIM", signed=False, parent=self)

class _Spi0_SyncConfig1(Register):
    ADDRESS = 0x48009018
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.STROBE_MASK      = Field(self.ADDRESS, self.WIDTH, 0x000001FF,  0, "SPI0.SYNC_CONFIG1.STROBE_MASK", signed=False, parent=self)
        self.SW_STROBE_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "SPI0.SYNC_CONFIG1.SW_STROBE_ENABLE", signed=False, parent=self)
        self.HW_STROBE_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "SPI0.SYNC_CONFIG1.HW_STROBE_ENABLE", signed=False, parent=self)
        self.SEC_CYCLE_ACTIVE = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "SPI0.SYNC_CONFIG1.SEC_CYCLE_ACTIVE", signed=False, parent=self)

class _Spi0_TransferConfig1(Register):
    ADDRESS = 0x4800901C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.SCLK_PHASE      = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "SPI0.TRANSFER_CONFIG1.SCLK_PHASE", signed=False, parent=self)
        self.SCLK_POL        = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "SPI0.TRANSFER_CONFIG1.SCLK_POL", signed=False, parent=self)
        self.CS_POL          = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "SPI0.TRANSFER_CONFIG1.CS_POL", signed=False, parent=self)
        self.CS0_ENABLE      = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "SPI0.TRANSFER_CONFIG1.CS0_ENABLE", signed=False, parent=self)
        self.CS1_ENABLE      = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "SPI0.TRANSFER_CONFIG1.CS1_ENABLE", signed=False, parent=self)
        self.CS2_ENABLE      = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "SPI0.TRANSFER_CONFIG1.CS2_ENABLE", signed=False, parent=self)
        self.APPLY_TAD       = Field(self.ADDRESS, self.WIDTH, 0x000000C0,  6, "SPI0.TRANSFER_CONFIG1.APPLY_TAD", signed=False, parent=self)
        self.DATA_SIZE_SEL_0 = Field(self.ADDRESS, self.WIDTH, 0x00000F00,  8, "SPI0.TRANSFER_CONFIG1.DATA_SIZE_SEL_0", signed=False, parent=self)
        self.DATA_SIZE_SEL_1 = Field(self.ADDRESS, self.WIDTH, 0x0000F000, 12, "SPI0.TRANSFER_CONFIG1.DATA_SIZE_SEL_1", signed=False, parent=self)

class _Spi0_SclkConfig2(Register):
    ADDRESS = 0x48009020
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.CLK_DIV       = Field(self.ADDRESS, self.WIDTH, 0x000000FE,  1, "SPI0.SCLK_CONFIG2.CLK_DIV", signed=False, parent=self)
        self.CS_SETTLE_TIM = Field(self.ADDRESS, self.WIDTH, 0x0000FF00,  8, "SPI0.SCLK_CONFIG2.CS_SETTLE_TIM", signed=False, parent=self)
        self.PAUSE_TIM     = Field(self.ADDRESS, self.WIDTH, 0x00FF0000, 16, "SPI0.SCLK_CONFIG2.PAUSE_TIM", signed=False, parent=self)

class _Spi0_SyncConfig2(Register):
    ADDRESS = 0x48009024
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.STROBE_MASK      = Field(self.ADDRESS, self.WIDTH, 0x000001FF,  0, "SPI0.SYNC_CONFIG2.STROBE_MASK", signed=False, parent=self)
        self.SW_STROBE_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "SPI0.SYNC_CONFIG2.SW_STROBE_ENABLE", signed=False, parent=self)
        self.HW_STROBE_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "SPI0.SYNC_CONFIG2.HW_STROBE_ENABLE", signed=False, parent=self)
        self.SEC_CYCLE_ACTIVE = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "SPI0.SYNC_CONFIG2.SEC_CYCLE_ACTIVE", signed=False, parent=self)

class _Spi0_TransferConfig2(Register):
    ADDRESS = 0x48009028
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.SCLK_PHASE      = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "SPI0.TRANSFER_CONFIG2.SCLK_PHASE", signed=False, parent=self)
        self.SCLK_POL        = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "SPI0.TRANSFER_CONFIG2.SCLK_POL", signed=False, parent=self)
        self.CS_POL          = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "SPI0.TRANSFER_CONFIG2.CS_POL", signed=False, parent=self)
        self.CS0_ENABLE      = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "SPI0.TRANSFER_CONFIG2.CS0_ENABLE", signed=False, parent=self)
        self.CS1_ENALBE      = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "SPI0.TRANSFER_CONFIG2.CS1_ENALBE", signed=False, parent=self)
        self.CS2_ENABLE      = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "SPI0.TRANSFER_CONFIG2.CS2_ENABLE", signed=False, parent=self)
        self.APPLY_TAD       = Field(self.ADDRESS, self.WIDTH, 0x000000C0,  6, "SPI0.TRANSFER_CONFIG2.APPLY_TAD", signed=False, parent=self)
        self.DATA_SIZE_SEL_0 = Field(self.ADDRESS, self.WIDTH, 0x00000F00,  8, "SPI0.TRANSFER_CONFIG2.DATA_SIZE_SEL_0", signed=False, parent=self)
        self.DATA_SIZE_SEL_1 = Field(self.ADDRESS, self.WIDTH, 0x0000F000, 12, "SPI0.TRANSFER_CONFIG2.DATA_SIZE_SEL_1", signed=False, parent=self)

class _Spi0_RdBuffer0(Register):
    ADDRESS = 0x4800902C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RD_BUFFER0 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPI0.RD_BUFFER0.RD_BUFFER0", signed=False, parent=self)

class _Spi0_RdBuffer1(Register):
    ADDRESS = 0x48009030
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RD_BUFFER1 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPI0.RD_BUFFER1.RD_BUFFER1", signed=False, parent=self)

class _Spi0_RdBuffer2(Register):
    ADDRESS = 0x48009034
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RD_BUFFER2 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPI0.RD_BUFFER2.RD_BUFFER2", signed=False, parent=self)

class _Spi0_RdBuffer3(Register):
    ADDRESS = 0x48009038
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RD_BUFFER3 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPI0.RD_BUFFER3.RD_BUFFER3", signed=False, parent=self)

class _Spi0_WrBuffer0(Register):
    ADDRESS = 0x4800903C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.WR_BUFFER0 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPI0.WR_BUFFER0.WR_BUFFER0", signed=False, parent=self)

class _Spi0_WrBuffer1(Register):
    ADDRESS = 0x48009040
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.WR_BUFFER1 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPI0.WR_BUFFER1.WR_BUFFER1", signed=False, parent=self)

class _Spi0_WrBuffer2(Register):
    ADDRESS = 0x48009044
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.WR_BUFFER2 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPI0.WR_BUFFER2.WR_BUFFER2", signed=False, parent=self)

class _Spi0_WrBuffer3(Register):
    ADDRESS = 0x48009048
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.WR_BUFFER3 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPI0.WR_BUFFER3.WR_BUFFER3", signed=False, parent=self)


class _Mcc(AddressBlock):
    _NAME       = "MCC"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.INFO_CHIP                        = _Mcc_InfoChip(self)
        self.INFO_VARIANT                     = _Mcc_InfoVariant(self)
        self.INFO_REVISION                    = _Mcc_InfoRevision(self)
        self.INFO_DATE                        = _Mcc_InfoDate(self)
        self.ADC_I1_I0_RAW                    = _Mcc_AdcI1I0Raw(self)
        self.ADC_I3_I2_RAW                    = _Mcc_AdcI3I2Raw(self)
        self.ADC_U1_U0_RAW                    = _Mcc_AdcU1U0Raw(self)
        self.ADC_U3_U2_RAW                    = _Mcc_AdcU3U2Raw(self)
        self.ADC_TEMP_VM_RAW                  = _Mcc_AdcTempVmRaw(self)
        self.ADC_AIN1_AIN0_RAW                = _Mcc_AdcAin1Ain0Raw(self)
        self.ADC_AIN3_AIN2_RAW                = _Mcc_AdcAin3Ain2Raw(self)
        self.ADC_I_GEN_CONFIG                 = _Mcc_AdcIGenConfig(self)
        self.ADC_I0_CONFIG                    = _Mcc_AdcI0Config(self)
        self.ADC_I1_CONFIG                    = _Mcc_AdcI1Config(self)
        self.ADC_I2_CONFIG                    = _Mcc_AdcI2Config(self)
        self.ADC_I3_CONFIG                    = _Mcc_AdcI3Config(self)
        self.ADC_I1_I0_SCALED                 = _Mcc_AdcI1I0Scaled(self)
        self.ADC_I3_I2_SCALED                 = _Mcc_AdcI3I2Scaled(self)
        self.ADC_IWY_IUX                      = _Mcc_AdcIwyIux(self)
        self.ADC_IV                           = _Mcc_AdcIv(self)
        self.ADC_STATUS                       = _Mcc_AdcStatus(self)
        self.MOTOR_CONFIG                     = _Mcc_MotorConfig(self)
        self.MOTION_CONFIG                    = _Mcc_MotionConfig(self)
        self.PHI_E_SELECTION                  = _Mcc_PhiESelection(self)
        self.PHI_E                            = _Mcc_PhiE(self)
        self.PWM_CONFIG                       = _Mcc_PwmConfig(self)
        self.PWM_MAXCNT                       = _Mcc_PwmMaxcnt(self)
        self.PWM_SWITCH_LIMIT                 = _Mcc_PwmSwitchLimit(self)
        self.PWM_WATCHDOG_CFG                 = _Mcc_PwmWatchdogCfg(self)
        self.ABN_PHI_E_PHI_M                  = _Mcc_AbnPhiEPhiM(self)
        self.ABN_MODE                         = _Mcc_AbnMode(self)
        self.ABN_CPR                          = _Mcc_AbnCpr(self)
        self.ABN_CPR_INV                      = _Mcc_AbnCprInv(self)
        self.ABN_COUNT                        = _Mcc_AbnCount(self)
        self.ABN_COUNT_N                      = _Mcc_AbnCountN(self)
        self.ABN_PHI_E_OFFSET                 = _Mcc_AbnPhiEOffset(self)
        self.HALL_MODE                        = _Mcc_HallMode(self)
        self.HALL_DPHI_MAX                    = _Mcc_HallDphiMax(self)
        self.HALL_PHI_E_OFFSET                = _Mcc_HallPhiEOffset(self)
        self.HALL_COUNT                       = _Mcc_HallCount(self)
        self.HALL_PHI_E_EXTRAPOLATED_PHI_E    = _Mcc_HallPhiEExtrapolatedPhiE(self)
        self.HALL_POSITION_060_POSITION_000   = _Mcc_HallPosition060Position000(self)
        self.HALL_POSITION_180_POSITION_120   = _Mcc_HallPosition180Position120(self)
        self.HALL_POSITION_300_POSITION_240   = _Mcc_HallPosition300Position240(self)
        self.BIQUAD_V_A_1                     = _Mcc_BiquadVA1(self)
        self.BIQUAD_V_A_2                     = _Mcc_BiquadVA2(self)
        self.BIQUAD_V_B_0                     = _Mcc_BiquadVB0(self)
        self.BIQUAD_V_B_1                     = _Mcc_BiquadVB1(self)
        self.BIQUAD_V_B_2                     = _Mcc_BiquadVB2(self)
        self.BIQUAD_V_ENABLE                  = _Mcc_BiquadVEnable(self)
        self.BIQUAD_T_A_1                     = _Mcc_BiquadTA1(self)
        self.BIQUAD_T_A_2                     = _Mcc_BiquadTA2(self)
        self.BIQUAD_T_B_0                     = _Mcc_BiquadTB0(self)
        self.BIQUAD_T_B_1                     = _Mcc_BiquadTB1(self)
        self.BIQUAD_T_B_2                     = _Mcc_BiquadTB2(self)
        self.BIQUAD_T_ENABLE                  = _Mcc_BiquadTEnable(self)
        self.VELOCITY_CONFIG                  = _Mcc_VelocityConfig(self)
        self.VELOCITY_SCALING                 = _Mcc_VelocityScaling(self)
        self.V_MIN_POS_DEV_TIME_COUNTER_LIMIT = _Mcc_VMinPosDevTimeCounterLimit(self)
        self.MAX_VEL_DEVIATION                = _Mcc_MaxVelDeviation(self)
        self.POSITION_CONFIG                  = _Mcc_PositionConfig(self)
        self.MAX_POS_DEVIATION                = _Mcc_MaxPosDeviation(self)
        self.POSITION_STEP_WIDTH              = _Mcc_PositionStepWidth(self)
        self.RAMPER_STATUS                    = _Mcc_RamperStatus(self)
        self.RAMPER_A1                        = _Mcc_RamperA1(self)
        self.RAMPER_A2                        = _Mcc_RamperA2(self)
        self.RAMPER_A_MAX                     = _Mcc_RamperAMax(self)
        self.RAMPER_D1                        = _Mcc_RamperD1(self)
        self.RAMPER_D2                        = _Mcc_RamperD2(self)
        self.RAMPER_D_MAX                     = _Mcc_RamperDMax(self)
        self.RAMPER_V_START                   = _Mcc_RamperVStart(self)
        self.RAMPER_V1                        = _Mcc_RamperV1(self)
        self.RAMPER_V2                        = _Mcc_RamperV2(self)
        self.RAMPER_V_STOP                    = _Mcc_RamperVStop(self)
        self.RAMPER_V_MAX                     = _Mcc_RamperVMax(self)
        self.RAMPER_V_TARGET                  = _Mcc_RamperVTarget(self)
        self.RAMPER_SWITCH_MODE               = _Mcc_RamperSwitchMode(self)
        self.RAMPER_TIME_CONFIG               = _Mcc_RamperTimeConfig(self)
        self.RAMPER_A_ACTUAL                  = _Mcc_RamperAActual(self)
        self.RAMPER_X_ACTUAL                  = _Mcc_RamperXActual(self)
        self.RAMPER_V_ACTUAL                  = _Mcc_RamperVActual(self)
        self.RAMPER_X_TARGET                  = _Mcc_RamperXTarget(self)
        self.RAMPER_PHI_E                     = _Mcc_RamperPhiE(self)
        self.RAMPER_PHI_E_OFFSET              = _Mcc_RamperPhiEOffset(self)
        self.RAMPER_ACC_FF                    = _Mcc_RamperAccFf(self)
        self.RAMPER_X_ACTUAL_LATCH            = _Mcc_RamperXActualLatch(self)
        self.POSITION_ACTUAL_LATCH            = _Mcc_PositionActualLatch(self)
        self.PRBS_AMPLITUDE                   = _Mcc_PrbsAmplitude(self)
        self.PRBS_DOWN_SAMPLING_RATIO         = _Mcc_PrbsDownSamplingRatio(self)
        self.PID_CONFIG                       = _Mcc_PidConfig(self)
        self.PID_FLUX_COEFF                   = _Mcc_PidFluxCoeff(self)
        self.PID_TORQUE_COEFF                 = _Mcc_PidTorqueCoeff(self)
        self.PID_FIELDWEAK_COEFF              = _Mcc_PidFieldweakCoeff(self)
        self.PID_U_S_MAX                      = _Mcc_PidUSMax(self)
        self.PID_VELOCITY_COEFF               = _Mcc_PidVelocityCoeff(self)
        self.PID_POSITION_COEFF               = _Mcc_PidPositionCoeff(self)
        self.PID_POSITION_TOLERANCE           = _Mcc_PidPositionTolerance(self)
        self.PID_POSITION_TOLERANCE_DELAY     = _Mcc_PidPositionToleranceDelay(self)
        self.PID_UQ_UD_LIMITS                 = _Mcc_PidUqUdLimits(self)
        self.PID_TORQUE_FLUX_LIMITS           = _Mcc_PidTorqueFluxLimits(self)
        self.PID_VELOCITY_LIMIT               = _Mcc_PidVelocityLimit(self)
        self.PID_POSITION_LIMIT_LOW           = _Mcc_PidPositionLimitLow(self)
        self.PID_POSITION_LIMIT_HIGH          = _Mcc_PidPositionLimitHigh(self)
        self.PID_TORQUE_FLUX_TARGET           = _Mcc_PidTorqueFluxTarget(self)
        self.PID_TORQUE_FLUX_OFFSET           = _Mcc_PidTorqueFluxOffset(self)
        self.PID_VELOCITY_TARGET              = _Mcc_PidVelocityTarget(self)
        self.PID_VELOCITY_OFFSET              = _Mcc_PidVelocityOffset(self)
        self.PID_POSITION_TARGET              = _Mcc_PidPositionTarget(self)
        self.PID_TORQUE_FLUX_ACTUAL           = _Mcc_PidTorqueFluxActual(self)
        self.PID_VELOCITY_ACTUAL              = _Mcc_PidVelocityActual(self)
        self.PID_POSITION_ACTUAL              = _Mcc_PidPositionActual(self)
        self.PID_POSITION_ACTUAL_OFFSET       = _Mcc_PidPositionActualOffset(self)
        self.PID_TORQUE_ERROR                 = _Mcc_PidTorqueError(self)
        self.PID_FLUX_ERROR                   = _Mcc_PidFluxError(self)
        self.PID_VELOCITY_ERROR               = _Mcc_PidVelocityError(self)
        self.PID_POSITION_ERROR               = _Mcc_PidPositionError(self)
        self.PID_TORQUE_INTEGRATOR            = _Mcc_PidTorqueIntegrator(self)
        self.PID_FLUX_INTEGRATOR              = _Mcc_PidFluxIntegrator(self)
        self.PID_VELOCITY_INTEGRATOR          = _Mcc_PidVelocityIntegrator(self)
        self.PID_POSITION_INTEGRATOR          = _Mcc_PidPositionIntegrator(self)
        self.PIDIN_TORQUE_FLUX_TARGET         = _Mcc_PidinTorqueFluxTarget(self)
        self.PIDIN_VELOCITY_TARGET            = _Mcc_PidinVelocityTarget(self)
        self.PIDIN_POSITION_TARGET            = _Mcc_PidinPositionTarget(self)
        self.PIDIN_TORQUE_FLUX_TARGET_LIMITED = _Mcc_PidinTorqueFluxTargetLimited(self)
        self.PIDIN_VELOCITY_TARGET_LIMITED    = _Mcc_PidinVelocityTargetLimited(self)
        self.PIDIN_POSITION_TARGET_LIMITED    = _Mcc_PidinPositionTargetLimited(self)
        self.FOC_IBETA_IALPHA                 = _Mcc_FocIbetaIalpha(self)
        self.FOC_IQ_ID                        = _Mcc_FocIqId(self)
        self.FOC_UQ_UD                        = _Mcc_FocUqUd(self)
        self.FOC_UQ_UD_LIMITED                = _Mcc_FocUqUdLimited(self)
        self.FOC_UBETA_UALPHA                 = _Mcc_FocUbetaUalpha(self)
        self.FOC_UWY_UUX                      = _Mcc_FocUwyUux(self)
        self.FOC_UV                           = _Mcc_FocUv(self)
        self.PWM_VX2_UX1                      = _Mcc_PwmVx2Ux1(self)
        self.PWM_Y2_WY1                       = _Mcc_PwmY2Wy1(self)
        self.VELOCITY_FRQ                     = _Mcc_VelocityFrq(self)
        self.VELOCITY_PER                     = _Mcc_VelocityPer(self)
        self.FOC_STATUS                       = _Mcc_FocStatus(self)
        self.U_S_ACTUAL_I_S_ACTUAL            = _Mcc_USActualISActual(self)
        self.P_MOTOR                          = _Mcc_PMotor(self)
        self.INPUTS_RAW                       = _Mcc_InputsRaw(self)
        self.OUTPUTS_RAW                      = _Mcc_OutputsRaw(self)
        self.STATUS_FLAGS                     = _Mcc_StatusFlags(self)
        self.STATUS_MASK                      = _Mcc_StatusMask(self)
        self.FLEX_COMP_CONF                   = _Mcc_FlexCompConf(self)
        self.FLEX_COMP_RESULT_V_U             = _Mcc_FlexCompResultVU(self)
        self.FLEX_COMP_RESULT_Y2_W            = _Mcc_FlexCompResultY2W(self)
        self.GDRV_HW                          = _Mcc_GdrvHw(self)
        self.GDRV_CFG                         = _Mcc_GdrvCfg(self)
        self.GDRV_TIMING                      = _Mcc_GdrvTiming(self)
        self.GDRV_BBM                         = _Mcc_GdrvBbm(self)
        self.GDRV_PROT                        = _Mcc_GdrvProt(self)
        self.GDRV_OCP_UVW                     = _Mcc_GdrvOcpUvw(self)
        self.GDRV_OCP_Y2                      = _Mcc_GdrvOcpY2(self)
        self.GDRV_PROT_EN                     = _Mcc_GdrvProtEn(self)
        self.GDRV_STATUS_EN                   = _Mcc_GdrvStatusEn(self)
        self.GDRV_STATUS                      = _Mcc_GdrvStatus(self)
        self.GDRV_FAULT                       = _Mcc_GdrvFault(self)
        self.ADC_I1_I0_EXT                    = _Mcc_AdcI1I0Ext(self)
        self.ADC_I2_EXT                       = _Mcc_AdcI2Ext(self)
        self.PWM_VX2_UX1_EXT                  = _Mcc_PwmVx2Ux1Ext(self)
        self.PWM_Y2_WY1_EXT                   = _Mcc_PwmY2Wy1Ext(self)
        self.PWM_EXT_Y2_ALT                   = _Mcc_PwmExtY2Alt(self)
        self.VOLTAGE_EXT                      = _Mcc_VoltageExt(self)
        self.PHI_EXT                          = _Mcc_PhiExt(self)
        self.VELOCITY_EXT                     = _Mcc_VelocityExt(self)


class _Mcc_InfoChip(Register):
    ADDRESS = 0x4800A000
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.ID     = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.INFO_CHIP.ID", signed=False, parent=self)
        self.PREFIX = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.INFO_CHIP.PREFIX", signed=False, parent=self)

class _Mcc_InfoVariant(Register):
    ADDRESS = 0x4800A004
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PMU_VAR     = Field(self.ADDRESS, self.WIDTH, 0x00000003,  0, "MCC.INFO_VARIANT.PMU_VAR", signed=False, parent=self)
        self.GDRV_VAR    = Field(self.ADDRESS, self.WIDTH, 0x0000000C,  2, "MCC.INFO_VARIANT.GDRV_VAR", signed=False, parent=self)
        self.ENABLE_JTAG = Field(self.ADDRESS, self.WIDTH, 0x80000000, 31, "MCC.INFO_VARIANT.ENABLE_JTAG", signed=False, parent=self)

class _Mcc_InfoRevision(Register):
    ADDRESS = 0x4800A008
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.REVISION = Field(self.ADDRESS, self.WIDTH, 0x7FFFFFFF,  0, "MCC.INFO_REVISION.REVISION", signed=False, parent=self)
        self.IS_FPGA  = Field(self.ADDRESS, self.WIDTH, 0x80000000, 31, "MCC.INFO_REVISION.IS_FPGA", signed=False, parent=self)

class _Mcc_InfoDate(Register):
    ADDRESS = 0x4800A00C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.MINUTE = Field(self.ADDRESS, self.WIDTH, 0x0000003F,  0, "MCC.INFO_DATE.MINUTE", signed=False, parent=self)
        self.HOUR   = Field(self.ADDRESS, self.WIDTH, 0x000007C0,  6, "MCC.INFO_DATE.HOUR", signed=False, parent=self)
        self.DAY    = Field(self.ADDRESS, self.WIDTH, 0x0000F800, 11, "MCC.INFO_DATE.DAY", signed=False, parent=self)
        self.MONTH  = Field(self.ADDRESS, self.WIDTH, 0x000F0000, 16, "MCC.INFO_DATE.MONTH", signed=False, parent=self)
        self.YEAR   = Field(self.ADDRESS, self.WIDTH, 0xFFF00000, 20, "MCC.INFO_DATE.YEAR", signed=False, parent=self)

class _Mcc_AdcI1I0Raw(Register):
    ADDRESS = 0x4800A080
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.I0 = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ADC_I1_I0_RAW.I0", signed=True, parent=self)
        self.I1 = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.ADC_I1_I0_RAW.I1", signed=True, parent=self)

class _Mcc_AdcI3I2Raw(Register):
    ADDRESS = 0x4800A084
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.I2 = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ADC_I3_I2_RAW.I2", signed=True, parent=self)
        self.I3 = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.ADC_I3_I2_RAW.I3", signed=True, parent=self)

class _Mcc_AdcU1U0Raw(Register):
    ADDRESS = 0x4800A088
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.U0 = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ADC_U1_U0_RAW.U0", signed=True, parent=self)
        self.U1 = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.ADC_U1_U0_RAW.U1", signed=True, parent=self)

class _Mcc_AdcU3U2Raw(Register):
    ADDRESS = 0x4800A08C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.U2 = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ADC_U3_U2_RAW.U2", signed=True, parent=self)
        self.U3 = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.ADC_U3_U2_RAW.U3", signed=True, parent=self)

class _Mcc_AdcTempVmRaw(Register):
    ADDRESS = 0x4800A090
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.VM   = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ADC_TEMP_VM_RAW.VM", signed=True, parent=self)
        self.TEMP = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.ADC_TEMP_VM_RAW.TEMP", signed=True, parent=self)

class _Mcc_AdcAin1Ain0Raw(Register):
    ADDRESS = 0x4800A094
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.AIN0 = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ADC_AIN1_AIN0_RAW.AIN0", signed=True, parent=self)
        self.AIN1 = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.ADC_AIN1_AIN0_RAW.AIN1", signed=True, parent=self)

class _Mcc_AdcAin3Ain2Raw(Register):
    ADDRESS = 0x4800A098
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.AIN2 = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ADC_AIN3_AIN2_RAW.AIN2", signed=True, parent=self)
        self.AIN3 = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.ADC_AIN3_AIN2_RAW.AIN3", signed=True, parent=self)

class _Mcc_AdcIGenConfig(Register):
    ADDRESS = 0x4800A100
    WIDTH = 4
    DEFAULT = 0x000010E4

    def __init__(self, parent):
        self._PARENT = parent

        self.UX1_SELECT       = Field(self.ADDRESS, self.WIDTH, 0x00000003,  0, "MCC.ADC_I_GEN_CONFIG.UX1_SELECT", signed=False, parent=self)
        self.VX2_SELECT       = Field(self.ADDRESS, self.WIDTH, 0x0000000C,  2, "MCC.ADC_I_GEN_CONFIG.VX2_SELECT", signed=False, parent=self)
        self.WY1_SELECT       = Field(self.ADDRESS, self.WIDTH, 0x00000030,  4, "MCC.ADC_I_GEN_CONFIG.WY1_SELECT", signed=False, parent=self)
        self.Y2_SELECT        = Field(self.ADDRESS, self.WIDTH, 0x000000C0,  6, "MCC.ADC_I_GEN_CONFIG.Y2_SELECT", signed=False, parent=self)
        self.SOURCE_SELECT    = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "MCC.ADC_I_GEN_CONFIG.SOURCE_SELECT", signed=False, parent=self)
        self.MEASUREMENT_MODE = Field(self.ADDRESS, self.WIDTH, 0x00000E00,  9, "MCC.ADC_I_GEN_CONFIG.MEASUREMENT_MODE", signed=False, parent=self)
        self.TRIGGER_SELECT   = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "MCC.ADC_I_GEN_CONFIG.TRIGGER_SELECT", signed=False, parent=self)
        self.TRIGGER_POS      = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.ADC_I_GEN_CONFIG.TRIGGER_POS", signed=False, parent=self)

class _Mcc_AdcI0Config(Register):
    ADDRESS = 0x4800A104
    WIDTH = 4
    DEFAULT = 0xFC000000

    def __init__(self, parent):
        self._PARENT = parent

        self.OFFSET = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ADC_I0_CONFIG.OFFSET", signed=True, parent=self)
        self.SCALE  = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.ADC_I0_CONFIG.SCALE", signed=True, parent=self)

class _Mcc_AdcI1Config(Register):
    ADDRESS = 0x4800A108
    WIDTH = 4
    DEFAULT = 0xFC000000

    def __init__(self, parent):
        self._PARENT = parent

        self.OFFSET = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ADC_I1_CONFIG.OFFSET", signed=True, parent=self)
        self.SCALE  = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.ADC_I1_CONFIG.SCALE", signed=True, parent=self)

class _Mcc_AdcI2Config(Register):
    ADDRESS = 0x4800A10C
    WIDTH = 4
    DEFAULT = 0xFC000000

    def __init__(self, parent):
        self._PARENT = parent

        self.OFFSET = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ADC_I2_CONFIG.OFFSET", signed=True, parent=self)
        self.SCALE  = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.ADC_I2_CONFIG.SCALE", signed=True, parent=self)

class _Mcc_AdcI3Config(Register):
    ADDRESS = 0x4800A110
    WIDTH = 4
    DEFAULT = 0xFC000000

    def __init__(self, parent):
        self._PARENT = parent

        self.OFFSET = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ADC_I3_CONFIG.OFFSET", signed=True, parent=self)
        self.SCALE  = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.ADC_I3_CONFIG.SCALE", signed=True, parent=self)

class _Mcc_AdcI1I0Scaled(Register):
    ADDRESS = 0x4800A114
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.I0 = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ADC_I1_I0_SCALED.I0", signed=True, parent=self)
        self.I1 = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.ADC_I1_I0_SCALED.I1", signed=True, parent=self)

class _Mcc_AdcI3I2Scaled(Register):
    ADDRESS = 0x4800A118
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.I2 = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ADC_I3_I2_SCALED.I2", signed=True, parent=self)
        self.I3 = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.ADC_I3_I2_SCALED.I3", signed=True, parent=self)

class _Mcc_AdcIwyIux(Register):
    ADDRESS = 0x4800A11C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.IUX = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ADC_IWY_IUX.IUX", signed=True, parent=self)
        self.IWY = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.ADC_IWY_IUX.IWY", signed=True, parent=self)

class _Mcc_AdcIv(Register):
    ADDRESS = 0x4800A120
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.IV = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ADC_IV.IV", signed=True, parent=self)

class _Mcc_AdcStatus(Register):
    ADDRESS = 0x4800A124
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.I0_CLIPPED   = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "MCC.ADC_STATUS.I0_CLIPPED", signed=False, parent=self)
        self.I1_CLIPPED   = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "MCC.ADC_STATUS.I1_CLIPPED", signed=False, parent=self)
        self.I2_CLIPPED   = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "MCC.ADC_STATUS.I2_CLIPPED", signed=False, parent=self)
        self.I3_CLIPPED   = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "MCC.ADC_STATUS.I3_CLIPPED", signed=False, parent=self)
        self.U0_CLIPPED   = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "MCC.ADC_STATUS.U0_CLIPPED", signed=False, parent=self)
        self.U1_CLIPPED   = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "MCC.ADC_STATUS.U1_CLIPPED", signed=False, parent=self)
        self.U2_CLIPPED   = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "MCC.ADC_STATUS.U2_CLIPPED", signed=False, parent=self)
        self.U3_CLIPPED   = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "MCC.ADC_STATUS.U3_CLIPPED", signed=False, parent=self)
        self.AIN0_CLIPPED = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "MCC.ADC_STATUS.AIN0_CLIPPED", signed=False, parent=self)
        self.AIN1_CLIPPED = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "MCC.ADC_STATUS.AIN1_CLIPPED", signed=False, parent=self)
        self.AIN2_CLIPPED = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "MCC.ADC_STATUS.AIN2_CLIPPED", signed=False, parent=self)
        self.AIN3_CLIPPED = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "MCC.ADC_STATUS.AIN3_CLIPPED", signed=False, parent=self)
        self.VM_CLIPPED   = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "MCC.ADC_STATUS.VM_CLIPPED", signed=False, parent=self)
        self.TEMP_CLIPPED = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "MCC.ADC_STATUS.TEMP_CLIPPED", signed=False, parent=self)
        self.I0_DONE      = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "MCC.ADC_STATUS.I0_DONE", signed=False, parent=self)
        self.I1_DONE      = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "MCC.ADC_STATUS.I1_DONE", signed=False, parent=self)
        self.I2_DONE      = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "MCC.ADC_STATUS.I2_DONE", signed=False, parent=self)
        self.I3_DONE      = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "MCC.ADC_STATUS.I3_DONE", signed=False, parent=self)
        self.U0_DONE      = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "MCC.ADC_STATUS.U0_DONE", signed=False, parent=self)
        self.U1_DONE      = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "MCC.ADC_STATUS.U1_DONE", signed=False, parent=self)
        self.U2_DONE      = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "MCC.ADC_STATUS.U2_DONE", signed=False, parent=self)
        self.U3_DONE      = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "MCC.ADC_STATUS.U3_DONE", signed=False, parent=self)
        self.AIN0_DONE    = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "MCC.ADC_STATUS.AIN0_DONE", signed=False, parent=self)
        self.AIN1_DONE    = Field(self.ADDRESS, self.WIDTH, 0x02000000, 25, "MCC.ADC_STATUS.AIN1_DONE", signed=False, parent=self)
        self.AIN2_DONE    = Field(self.ADDRESS, self.WIDTH, 0x04000000, 26, "MCC.ADC_STATUS.AIN2_DONE", signed=False, parent=self)
        self.AIN3_DONE    = Field(self.ADDRESS, self.WIDTH, 0x08000000, 27, "MCC.ADC_STATUS.AIN3_DONE", signed=False, parent=self)
        self.VM_DONE      = Field(self.ADDRESS, self.WIDTH, 0x10000000, 28, "MCC.ADC_STATUS.VM_DONE", signed=False, parent=self)
        self.TEMP_DONE    = Field(self.ADDRESS, self.WIDTH, 0x20000000, 29, "MCC.ADC_STATUS.TEMP_DONE", signed=False, parent=self)

class _Mcc_MotorConfig(Register):
    ADDRESS = 0x4800A180
    WIDTH = 4
    DEFAULT = 0x00000001

    def __init__(self, parent):
        self._PARENT = parent

        self.N_POLE_PAIRS = Field(self.ADDRESS, self.WIDTH, 0x0000007F,  0, "MCC.MOTOR_CONFIG.N_POLE_PAIRS", signed=False, parent=self)
        self.TYPE         = Field(self.ADDRESS, self.WIDTH, 0x00030000, 16, "MCC.MOTOR_CONFIG.TYPE", signed=False, parent=self)

class _Mcc_MotionConfig(Register):
    ADDRESS = 0x4800A184
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.MOTION_MODE = Field(self.ADDRESS, self.WIDTH, 0x0000000F,  0, "MCC.MOTION_CONFIG.MOTION_MODE", signed=False, parent=self)
        self.RAMP_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "MCC.MOTION_CONFIG.RAMP_ENABLE", signed=False, parent=self)
        self.RAMP_MODE   = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "MCC.MOTION_CONFIG.RAMP_MODE", signed=False, parent=self)
        self.FEEDFORWARD = Field(self.ADDRESS, self.WIDTH, 0x000000C0,  6, "MCC.MOTION_CONFIG.FEEDFORWARD", signed=False, parent=self)

class _Mcc_PhiESelection(Register):
    ADDRESS = 0x4800A188
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PHI_E_SELECTION = Field(self.ADDRESS, self.WIDTH, 0x0000000F,  0, "MCC.PHI_E_SELECTION.PHI_E_SELECTION", signed=False, parent=self)

class _Mcc_PhiE(Register):
    ADDRESS = 0x4800A18C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PHI_E = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PHI_E.PHI_E", signed=True, parent=self)

class _Mcc_PwmConfig(Register):
    ADDRESS = 0x4800A200
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.CHOP              = Field(self.ADDRESS, self.WIDTH, 0x00000007,  0, "MCC.PWM_CONFIG.CHOP", signed=False, parent=self)
        self.SV_MODE           = Field(self.ADDRESS, self.WIDTH, 0x00000030,  4, "MCC.PWM_CONFIG.SV_MODE", signed=False, parent=self)
        self.Y2_HS_SRC         = Field(self.ADDRESS, self.WIDTH, 0x000000C0,  6, "MCC.PWM_CONFIG.Y2_HS_SRC", signed=False, parent=self)
        self.ENABLE_UX1        = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "MCC.PWM_CONFIG.ENABLE_UX1", signed=False, parent=self)
        self.ENABLE_VX2        = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "MCC.PWM_CONFIG.ENABLE_VX2", signed=False, parent=self)
        self.ENABLE_WY1        = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "MCC.PWM_CONFIG.ENABLE_WY1", signed=False, parent=self)
        self.ENABLE_Y2         = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "MCC.PWM_CONFIG.ENABLE_Y2", signed=False, parent=self)
        self.EXT_ENABLE_UX1    = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "MCC.PWM_CONFIG.EXT_ENABLE_UX1", signed=False, parent=self)
        self.EXT_ENABLE_VX2    = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "MCC.PWM_CONFIG.EXT_ENABLE_VX2", signed=False, parent=self)
        self.EXT_ENABLE_WY1    = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "MCC.PWM_CONFIG.EXT_ENABLE_WY1", signed=False, parent=self)
        self.EXT_ENABLE_Y2     = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "MCC.PWM_CONFIG.EXT_ENABLE_Y2", signed=False, parent=self)
        self.DUTY_CYCLE_OFFSET = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.PWM_CONFIG.DUTY_CYCLE_OFFSET", signed=False, parent=self)

class _Mcc_PwmMaxcnt(Register):
    ADDRESS = 0x4800A204
    WIDTH = 4
    DEFAULT = 0x000012BF

    def __init__(self, parent):
        self._PARENT = parent

        self.PWM_MAXCNT = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PWM_MAXCNT.PWM_MAXCNT", signed=False, parent=self)

class _Mcc_PwmSwitchLimit(Register):
    ADDRESS = 0x4800A20C
    WIDTH = 4
    DEFAULT = 0x0000FFFF

    def __init__(self, parent):
        self._PARENT = parent

        self.PWM_SWITCH_LIMIT = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PWM_SWITCH_LIMIT.PWM_SWITCH_LIMIT", signed=False, parent=self)

class _Mcc_PwmWatchdogCfg(Register):
    ADDRESS = 0x4800A210
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.ENABLE  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "MCC.PWM_WATCHDOG_CFG.ENABLE", signed=False, parent=self)
        self.DIV2    = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "MCC.PWM_WATCHDOG_CFG.DIV2", signed=False, parent=self)
        self.TRIGGER = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "MCC.PWM_WATCHDOG_CFG.TRIGGER", signed=False, parent=self)

class _Mcc_AbnPhiEPhiM(Register):
    ADDRESS = 0x4800A280
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PHI_M = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ABN_PHI_E_PHI_M.PHI_M", signed=True, parent=self)
        self.PHI_E = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.ABN_PHI_E_PHI_M.PHI_E", signed=True, parent=self)

class _Mcc_AbnMode(Register):
    ADDRESS = 0x4800A284
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.A_POL            = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "MCC.ABN_MODE.A_POL", signed=False, parent=self)
        self.B_POL            = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "MCC.ABN_MODE.B_POL", signed=False, parent=self)
        self.N_POL            = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "MCC.ABN_MODE.N_POL", signed=False, parent=self)
        self.COMBINED_N       = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "MCC.ABN_MODE.COMBINED_N", signed=False, parent=self)
        self.CLEAR_COUNT_ON_N = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "MCC.ABN_MODE.CLEAR_COUNT_ON_N", signed=False, parent=self)
        self.DISABLE_FILTER   = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "MCC.ABN_MODE.DISABLE_FILTER", signed=False, parent=self)
        self.CLN              = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "MCC.ABN_MODE.CLN", signed=False, parent=self)
        self.DIRECTION        = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "MCC.ABN_MODE.DIRECTION", signed=False, parent=self)

class _Mcc_AbnCpr(Register):
    ADDRESS = 0x4800A288
    WIDTH = 4
    DEFAULT = 0x00010000

    def __init__(self, parent):
        self._PARENT = parent

        self.ABN_CPR = Field(self.ADDRESS, self.WIDTH, 0x00FFFFFF,  0, "MCC.ABN_CPR.ABN_CPR", signed=False, parent=self)

class _Mcc_AbnCprInv(Register):
    ADDRESS = 0x4800A28C
    WIDTH = 4
    DEFAULT = 0x00010000

    def __init__(self, parent):
        self._PARENT = parent

        self.ABN_CPR_INV = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.ABN_CPR_INV.ABN_CPR_INV", signed=False, parent=self)

class _Mcc_AbnCount(Register):
    ADDRESS = 0x4800A290
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.ABN_COUNT = Field(self.ADDRESS, self.WIDTH, 0x00FFFFFF,  0, "MCC.ABN_COUNT.ABN_COUNT", signed=False, parent=self)

class _Mcc_AbnCountN(Register):
    ADDRESS = 0x4800A294
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.ABN_COUNT_N = Field(self.ADDRESS, self.WIDTH, 0x00FFFFFF,  0, "MCC.ABN_COUNT_N.ABN_COUNT_N", signed=False, parent=self)

class _Mcc_AbnPhiEOffset(Register):
    ADDRESS = 0x4800A298
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.ABN_PHI_E_OFFSET = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ABN_PHI_E_OFFSET.ABN_PHI_E_OFFSET", signed=True, parent=self)

class _Mcc_HallMode(Register):
    ADDRESS = 0x4800A300
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.POLARITY      = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "MCC.HALL_MODE.POLARITY", signed=False, parent=self)
        self.EXTRAPOLATION = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "MCC.HALL_MODE.EXTRAPOLATION", signed=False, parent=self)
        self.ORDER         = Field(self.ADDRESS, self.WIDTH, 0x00000070,  4, "MCC.HALL_MODE.ORDER", signed=False, parent=self)
        self.FILTER        = Field(self.ADDRESS, self.WIDTH, 0x0000FF00,  8, "MCC.HALL_MODE.FILTER", signed=False, parent=self)

class _Mcc_HallDphiMax(Register):
    ADDRESS = 0x4800A304
    WIDTH = 4
    DEFAULT = 0x00002AAA

    def __init__(self, parent):
        self._PARENT = parent

        self.HALL_DPHI_MAX = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.HALL_DPHI_MAX.HALL_DPHI_MAX", signed=False, parent=self)

class _Mcc_HallPhiEOffset(Register):
    ADDRESS = 0x4800A308
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.HALL_PHI_E_OFFSET = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.HALL_PHI_E_OFFSET.HALL_PHI_E_OFFSET", signed=True, parent=self)

class _Mcc_HallCount(Register):
    ADDRESS = 0x4800A30C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.HALL_COUNT = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.HALL_COUNT.HALL_COUNT", signed=True, parent=self)

class _Mcc_HallPhiEExtrapolatedPhiE(Register):
    ADDRESS = 0x4800A310
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PHI_E              = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.HALL_PHI_E_EXTRAPOLATED_PHI_E.PHI_E", signed=True, parent=self)
        self.PHI_E_EXTRAPOLATED = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.HALL_PHI_E_EXTRAPOLATED_PHI_E.PHI_E_EXTRAPOLATED", signed=True, parent=self)

class _Mcc_HallPosition060Position000(Register):
    ADDRESS = 0x4800A314
    WIDTH = 4
    DEFAULT = 0x2AAA0000

    def __init__(self, parent):
        self._PARENT = parent

        self.POSITION_000 = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.HALL_POSITION_060_POSITION_000.POSITION_000", signed=True, parent=self)
        self.POSITION_060 = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.HALL_POSITION_060_POSITION_000.POSITION_060", signed=True, parent=self)

class _Mcc_HallPosition180Position120(Register):
    ADDRESS = 0x4800A318
    WIDTH = 4
    DEFAULT = 0x80005555

    def __init__(self, parent):
        self._PARENT = parent

        self.POSITION_120 = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.HALL_POSITION_180_POSITION_120.POSITION_120", signed=True, parent=self)
        self.POSITION_180 = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.HALL_POSITION_180_POSITION_120.POSITION_180", signed=True, parent=self)

class _Mcc_HallPosition300Position240(Register):
    ADDRESS = 0x4800A31C
    WIDTH = 4
    DEFAULT = 0xD555AAAA

    def __init__(self, parent):
        self._PARENT = parent

        self.POSITION_240 = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.HALL_POSITION_300_POSITION_240.POSITION_240", signed=True, parent=self)
        self.POSITION_300 = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.HALL_POSITION_300_POSITION_240.POSITION_300", signed=True, parent=self)

class _Mcc_BiquadVA1(Register):
    ADDRESS = 0x4800A380
    WIDTH = 4
    DEFAULT = 0x001C376B

    def __init__(self, parent):
        self._PARENT = parent

        self.BIQUAD_V_A_1 = Field(self.ADDRESS, self.WIDTH, 0x00FFFFFF,  0, "MCC.BIQUAD_V_A_1.BIQUAD_V_A_1", signed=True, parent=self)

class _Mcc_BiquadVA2(Register):
    ADDRESS = 0x4800A384
    WIDTH = 4
    DEFAULT = 0x00F38F52

    def __init__(self, parent):
        self._PARENT = parent

        self.BIQUAD_V_A_2 = Field(self.ADDRESS, self.WIDTH, 0x00FFFFFF,  0, "MCC.BIQUAD_V_A_2.BIQUAD_V_A_2", signed=True, parent=self)

class _Mcc_BiquadVB0(Register):
    ADDRESS = 0x4800A388
    WIDTH = 4
    DEFAULT = 0x00000E51

    def __init__(self, parent):
        self._PARENT = parent

        self.BIQUAD_V_B_0 = Field(self.ADDRESS, self.WIDTH, 0x00FFFFFF,  0, "MCC.BIQUAD_V_B_0.BIQUAD_V_B_0", signed=True, parent=self)

class _Mcc_BiquadVB1(Register):
    ADDRESS = 0x4800A38C
    WIDTH = 4
    DEFAULT = 0x00001CA1

    def __init__(self, parent):
        self._PARENT = parent

        self.BIQUAD_V_B_1 = Field(self.ADDRESS, self.WIDTH, 0x00FFFFFF,  0, "MCC.BIQUAD_V_B_1.BIQUAD_V_B_1", signed=True, parent=self)

class _Mcc_BiquadVB2(Register):
    ADDRESS = 0x4800A390
    WIDTH = 4
    DEFAULT = 0x00000E51

    def __init__(self, parent):
        self._PARENT = parent

        self.BIQUAD_V_B_2 = Field(self.ADDRESS, self.WIDTH, 0x00FFFFFF,  0, "MCC.BIQUAD_V_B_2.BIQUAD_V_B_2", signed=True, parent=self)

class _Mcc_BiquadVEnable(Register):
    ADDRESS = 0x4800A394
    WIDTH = 4
    DEFAULT = 0x00000001

    def __init__(self, parent):
        self._PARENT = parent

        self.BIQUAD_V_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "MCC.BIQUAD_V_ENABLE.BIQUAD_V_ENABLE", signed=False, parent=self)

class _Mcc_BiquadTA1(Register):
    ADDRESS = 0x4800A398
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.BIQUAD_T_A_1 = Field(self.ADDRESS, self.WIDTH, 0x00FFFFFF,  0, "MCC.BIQUAD_T_A_1.BIQUAD_T_A_1", signed=True, parent=self)

class _Mcc_BiquadTA2(Register):
    ADDRESS = 0x4800A39C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.BIQUAD_T_A_2 = Field(self.ADDRESS, self.WIDTH, 0x00FFFFFF,  0, "MCC.BIQUAD_T_A_2.BIQUAD_T_A_2", signed=True, parent=self)

class _Mcc_BiquadTB0(Register):
    ADDRESS = 0x4800A3A0
    WIDTH = 4
    DEFAULT = 0x00100000

    def __init__(self, parent):
        self._PARENT = parent

        self.BIQUAD_T_B_0 = Field(self.ADDRESS, self.WIDTH, 0x00FFFFFF,  0, "MCC.BIQUAD_T_B_0.BIQUAD_T_B_0", signed=True, parent=self)

class _Mcc_BiquadTB1(Register):
    ADDRESS = 0x4800A3A4
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.BIQUAD_T_B_1 = Field(self.ADDRESS, self.WIDTH, 0x00FFFFFF,  0, "MCC.BIQUAD_T_B_1.BIQUAD_T_B_1", signed=True, parent=self)

class _Mcc_BiquadTB2(Register):
    ADDRESS = 0x4800A3A8
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.BIQUAD_T_B_2 = Field(self.ADDRESS, self.WIDTH, 0x00FFFFFF,  0, "MCC.BIQUAD_T_B_2.BIQUAD_T_B_2", signed=True, parent=self)

class _Mcc_BiquadTEnable(Register):
    ADDRESS = 0x4800A3AC
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.BIQUAD_T_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "MCC.BIQUAD_T_ENABLE.BIQUAD_T_ENABLE", signed=False, parent=self)

class _Mcc_VelocityConfig(Register):
    ADDRESS = 0x4800A400
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.SELECTION                  = Field(self.ADDRESS, self.WIDTH, 0x000000FF,  0, "MCC.VELOCITY_CONFIG.SELECTION", signed=False, parent=self)
        self.METER_SYNC_PULSE           = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "MCC.VELOCITY_CONFIG.METER_SYNC_PULSE", signed=False, parent=self)
        self.METER_TYPE                 = Field(self.ADDRESS, self.WIDTH, 0x00000600,  9, "MCC.VELOCITY_CONFIG.METER_TYPE", signed=False, parent=self)
        self.MOVING_AVRG_FILTER_SAMPLES = Field(self.ADDRESS, self.WIDTH, 0x00007000, 12, "MCC.VELOCITY_CONFIG.MOVING_AVRG_FILTER_SAMPLES", signed=False, parent=self)

class _Mcc_VelocityScaling(Register):
    ADDRESS = 0x4800A404
    WIDTH = 4
    DEFAULT = 0x000028F6

    def __init__(self, parent):
        self._PARENT = parent

        self.VELOCITY_SCALING = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.VELOCITY_SCALING.VELOCITY_SCALING", signed=True, parent=self)

class _Mcc_VMinPosDevTimeCounterLimit(Register):
    ADDRESS = 0x4800A408
    WIDTH = 4
    DEFAULT = 0x0001FFF0

    def __init__(self, parent):
        self._PARENT = parent

        self.TIME_COUNTER_LIMIT = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.V_MIN_POS_DEV_TIME_COUNTER_LIMIT.TIME_COUNTER_LIMIT", signed=False, parent=self)
        self.V_MIN_POS_DEV      = Field(self.ADDRESS, self.WIDTH, 0x7FFF0000, 16, "MCC.V_MIN_POS_DEV_TIME_COUNTER_LIMIT.V_MIN_POS_DEV", signed=False, parent=self)

class _Mcc_MaxVelDeviation(Register):
    ADDRESS = 0x4800A40C
    WIDTH = 4
    DEFAULT = 0x00010000

    def __init__(self, parent):
        self._PARENT = parent

        self.MAX_VEL_DEVIATION = Field(self.ADDRESS, self.WIDTH, 0x7FFFFFFF,  0, "MCC.MAX_VEL_DEVIATION.MAX_VEL_DEVIATION", signed=False, parent=self)

class _Mcc_PositionConfig(Register):
    ADDRESS = 0x4800A480
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.SELECTION = Field(self.ADDRESS, self.WIDTH, 0x000000FF,  0, "MCC.POSITION_CONFIG.SELECTION", signed=False, parent=self)

class _Mcc_MaxPosDeviation(Register):
    ADDRESS = 0x4800A484
    WIDTH = 4
    DEFAULT = 0x00008000

    def __init__(self, parent):
        self._PARENT = parent

        self.MAX_POS_DEVIATION = Field(self.ADDRESS, self.WIDTH, 0x7FFFFFFF,  0, "MCC.MAX_POS_DEVIATION.MAX_POS_DEVIATION", signed=False, parent=self)

class _Mcc_PositionStepWidth(Register):
    ADDRESS = 0x4800A488
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.POSITION_STEP_WIDTH = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.POSITION_STEP_WIDTH.POSITION_STEP_WIDTH", signed=False, parent=self)

class _Mcc_RamperStatus(Register):
    ADDRESS = 0x4800A500
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.STATUS_STOP_L     = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "MCC.RAMPER_STATUS.STATUS_STOP_L", signed=False, parent=self)
        self.STATUS_STOP_R     = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "MCC.RAMPER_STATUS.STATUS_STOP_R", signed=False, parent=self)
        self.STATUS_STOP_H     = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "MCC.RAMPER_STATUS.STATUS_STOP_H", signed=False, parent=self)
        self.STATUS_LATCH_L    = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "MCC.RAMPER_STATUS.STATUS_LATCH_L", signed=False, parent=self)
        self.STATUS_LATCH_R    = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "MCC.RAMPER_STATUS.STATUS_LATCH_R", signed=False, parent=self)
        self.STATUS_LATCH_H    = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "MCC.RAMPER_STATUS.STATUS_LATCH_H", signed=False, parent=self)
        self.EVENT_STOP_L      = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "MCC.RAMPER_STATUS.EVENT_STOP_L", signed=False, parent=self)
        self.EVENT_STOP_R      = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "MCC.RAMPER_STATUS.EVENT_STOP_R", signed=False, parent=self)
        self.EVENT_STOP_H      = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "MCC.RAMPER_STATUS.EVENT_STOP_H", signed=False, parent=self)
        self.EVENT_STOP_SG     = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "MCC.RAMPER_STATUS.EVENT_STOP_SG", signed=False, parent=self)
        self.EVENT_POS_REACHED = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "MCC.RAMPER_STATUS.EVENT_POS_REACHED", signed=False, parent=self)
        self.VELOCITY_REACHED  = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "MCC.RAMPER_STATUS.VELOCITY_REACHED", signed=False, parent=self)
        self.POSITION_REACHED  = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "MCC.RAMPER_STATUS.POSITION_REACHED", signed=False, parent=self)
        self.V_ZERO            = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "MCC.RAMPER_STATUS.V_ZERO", signed=False, parent=self)
        self.T_ZEROWAIT_ACTIVE = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "MCC.RAMPER_STATUS.T_ZEROWAIT_ACTIVE", signed=False, parent=self)
        self.SECOND_MOVE       = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "MCC.RAMPER_STATUS.SECOND_MOVE", signed=False, parent=self)
        self.STALL_IN_VEL_ERR  = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "MCC.RAMPER_STATUS.STALL_IN_VEL_ERR", signed=False, parent=self)
        self.STALL_IN_POS_ERR  = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "MCC.RAMPER_STATUS.STALL_IN_POS_ERR", signed=False, parent=self)

class _Mcc_RamperA1(Register):
    ADDRESS = 0x4800A504
    WIDTH = 4
    DEFAULT = 0x00010000

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_A1 = Field(self.ADDRESS, self.WIDTH, 0x007FFFFF,  0, "MCC.RAMPER_A1.RAMPER_A1", signed=False, parent=self)

class _Mcc_RamperA2(Register):
    ADDRESS = 0x4800A508
    WIDTH = 4
    DEFAULT = 0x00010000

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_A2 = Field(self.ADDRESS, self.WIDTH, 0x007FFFFF,  0, "MCC.RAMPER_A2.RAMPER_A2", signed=False, parent=self)

class _Mcc_RamperAMax(Register):
    ADDRESS = 0x4800A50C
    WIDTH = 4
    DEFAULT = 0x00010000

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_A_MAX = Field(self.ADDRESS, self.WIDTH, 0x007FFFFF,  0, "MCC.RAMPER_A_MAX.RAMPER_A_MAX", signed=False, parent=self)

class _Mcc_RamperD1(Register):
    ADDRESS = 0x4800A510
    WIDTH = 4
    DEFAULT = 0x00010000

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_D1 = Field(self.ADDRESS, self.WIDTH, 0x007FFFFF,  0, "MCC.RAMPER_D1.RAMPER_D1", signed=False, parent=self)

class _Mcc_RamperD2(Register):
    ADDRESS = 0x4800A514
    WIDTH = 4
    DEFAULT = 0x00010000

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_D2 = Field(self.ADDRESS, self.WIDTH, 0x007FFFFF,  0, "MCC.RAMPER_D2.RAMPER_D2", signed=False, parent=self)

class _Mcc_RamperDMax(Register):
    ADDRESS = 0x4800A518
    WIDTH = 4
    DEFAULT = 0x00010000

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_D_MAX = Field(self.ADDRESS, self.WIDTH, 0x007FFFFF,  0, "MCC.RAMPER_D_MAX.RAMPER_D_MAX", signed=False, parent=self)

class _Mcc_RamperVStart(Register):
    ADDRESS = 0x4800A51C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_V_START = Field(self.ADDRESS, self.WIDTH, 0x007FFFFF,  0, "MCC.RAMPER_V_START.RAMPER_V_START", signed=False, parent=self)

class _Mcc_RamperV1(Register):
    ADDRESS = 0x4800A520
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_V1 = Field(self.ADDRESS, self.WIDTH, 0x07FFFFFF,  0, "MCC.RAMPER_V1.RAMPER_V1", signed=False, parent=self)

class _Mcc_RamperV2(Register):
    ADDRESS = 0x4800A524
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_V2 = Field(self.ADDRESS, self.WIDTH, 0x07FFFFFF,  0, "MCC.RAMPER_V2.RAMPER_V2", signed=False, parent=self)

class _Mcc_RamperVStop(Register):
    ADDRESS = 0x4800A528
    WIDTH = 4
    DEFAULT = 0x00000100

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_V_STOP = Field(self.ADDRESS, self.WIDTH, 0x007FFFFF,  0, "MCC.RAMPER_V_STOP.RAMPER_V_STOP", signed=False, parent=self)

class _Mcc_RamperVMax(Register):
    ADDRESS = 0x4800A52C
    WIDTH = 4
    DEFAULT = 0x07FFFFFF

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_V_MAX = Field(self.ADDRESS, self.WIDTH, 0x07FFFFFF,  0, "MCC.RAMPER_V_MAX.RAMPER_V_MAX", signed=False, parent=self)

class _Mcc_RamperVTarget(Register):
    ADDRESS = 0x4800A530
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_V_TARGET = Field(self.ADDRESS, self.WIDTH, 0x0FFFFFFF,  0, "MCC.RAMPER_V_TARGET.RAMPER_V_TARGET", signed=True, parent=self)

class _Mcc_RamperSwitchMode(Register):
    ADDRESS = 0x4800A534
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.STOP_L_ENABLE         = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "MCC.RAMPER_SWITCH_MODE.STOP_L_ENABLE", signed=False, parent=self)
        self.STOP_R_ENABLE         = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "MCC.RAMPER_SWITCH_MODE.STOP_R_ENABLE", signed=False, parent=self)
        self.STOP_H_ENABLE         = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "MCC.RAMPER_SWITCH_MODE.STOP_H_ENABLE", signed=False, parent=self)
        self.STOP_L_POL            = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "MCC.RAMPER_SWITCH_MODE.STOP_L_POL", signed=False, parent=self)
        self.STOP_R_POL            = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "MCC.RAMPER_SWITCH_MODE.STOP_R_POL", signed=False, parent=self)
        self.STOP_H_POL            = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "MCC.RAMPER_SWITCH_MODE.STOP_H_POL", signed=False, parent=self)
        self.SWAP_LR               = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "MCC.RAMPER_SWITCH_MODE.SWAP_LR", signed=False, parent=self)
        self.LATCH_L_ACTIVE        = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "MCC.RAMPER_SWITCH_MODE.LATCH_L_ACTIVE", signed=False, parent=self)
        self.LATCH_L_INACTIVE      = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "MCC.RAMPER_SWITCH_MODE.LATCH_L_INACTIVE", signed=False, parent=self)
        self.LATCH_R_ACTIVE        = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "MCC.RAMPER_SWITCH_MODE.LATCH_R_ACTIVE", signed=False, parent=self)
        self.LATCH_R_INACTIVE      = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "MCC.RAMPER_SWITCH_MODE.LATCH_R_INACTIVE", signed=False, parent=self)
        self.LATCH_H_ACTIVE        = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "MCC.RAMPER_SWITCH_MODE.LATCH_H_ACTIVE", signed=False, parent=self)
        self.LATCH_H_INACTIVE      = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "MCC.RAMPER_SWITCH_MODE.LATCH_H_INACTIVE", signed=False, parent=self)
        self.SG_STOP_ENABLE        = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "MCC.RAMPER_SWITCH_MODE.SG_STOP_ENABLE", signed=False, parent=self)
        self.SOFTSTOP_ENABLE       = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "MCC.RAMPER_SWITCH_MODE.SOFTSTOP_ENABLE", signed=False, parent=self)
        self.SW_HARD_STOP          = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "MCC.RAMPER_SWITCH_MODE.SW_HARD_STOP", signed=False, parent=self)
        self.STOP_ON_POS_DEVIATION = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "MCC.RAMPER_SWITCH_MODE.STOP_ON_POS_DEVIATION", signed=False, parent=self)
        self.STOP_ON_VEL_DEVIATION = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "MCC.RAMPER_SWITCH_MODE.STOP_ON_VEL_DEVIATION", signed=False, parent=self)
        self.VELOCITY_OVERWRITE    = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "MCC.RAMPER_SWITCH_MODE.VELOCITY_OVERWRITE", signed=False, parent=self)

class _Mcc_RamperTimeConfig(Register):
    ADDRESS = 0x4800A538
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.T_ZEROWAIT = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.RAMPER_TIME_CONFIG.T_ZEROWAIT", signed=False, parent=self)
        self.T_VMAX     = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.RAMPER_TIME_CONFIG.T_VMAX", signed=False, parent=self)

class _Mcc_RamperAActual(Register):
    ADDRESS = 0x4800A53C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_A_ACTUAL = Field(self.ADDRESS, self.WIDTH, 0x00FFFFFF,  0, "MCC.RAMPER_A_ACTUAL.RAMPER_A_ACTUAL", signed=True, parent=self)

class _Mcc_RamperXActual(Register):
    ADDRESS = 0x4800A540
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_X_ACTUAL = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.RAMPER_X_ACTUAL.RAMPER_X_ACTUAL", signed=True, parent=self)

class _Mcc_RamperVActual(Register):
    ADDRESS = 0x4800A544
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_V_ACTUAL = Field(self.ADDRESS, self.WIDTH, 0x0FFFFFFF,  0, "MCC.RAMPER_V_ACTUAL.RAMPER_V_ACTUAL", signed=True, parent=self)

class _Mcc_RamperXTarget(Register):
    ADDRESS = 0x4800A548
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_X_TARGET = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.RAMPER_X_TARGET.RAMPER_X_TARGET", signed=True, parent=self)

class _Mcc_RamperPhiE(Register):
    ADDRESS = 0x4800A54C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_PHI_E = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.RAMPER_PHI_E.RAMPER_PHI_E", signed=True, parent=self)

class _Mcc_RamperPhiEOffset(Register):
    ADDRESS = 0x4800A550
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_PHI_E_OFFSET = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.RAMPER_PHI_E_OFFSET.RAMPER_PHI_E_OFFSET", signed=True, parent=self)

class _Mcc_RamperAccFf(Register):
    ADDRESS = 0x4800A554
    WIDTH = 4
    DEFAULT = 0x00060000

    def __init__(self, parent):
        self._PARENT = parent

        self.GAIN  = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.RAMPER_ACC_FF.GAIN", signed=False, parent=self)
        self.SHIFT = Field(self.ADDRESS, self.WIDTH, 0x00070000, 16, "MCC.RAMPER_ACC_FF.SHIFT", signed=False, parent=self)

class _Mcc_RamperXActualLatch(Register):
    ADDRESS = 0x4800A558
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RAMPER_X_ACTUAL_LATCH = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.RAMPER_X_ACTUAL_LATCH.RAMPER_X_ACTUAL_LATCH", signed=True, parent=self)

class _Mcc_PositionActualLatch(Register):
    ADDRESS = 0x4800A55C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.POSITION_ACTUAL_LATCH = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.POSITION_ACTUAL_LATCH.POSITION_ACTUAL_LATCH", signed=True, parent=self)

class _Mcc_PrbsAmplitude(Register):
    ADDRESS = 0x4800A580
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PRBS_AMPLITUDE = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PRBS_AMPLITUDE.PRBS_AMPLITUDE", signed=True, parent=self)

class _Mcc_PrbsDownSamplingRatio(Register):
    ADDRESS = 0x4800A584
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PRBS_DOWN_SAMPLING_RATIO = Field(self.ADDRESS, self.WIDTH, 0x000000FF,  0, "MCC.PRBS_DOWN_SAMPLING_RATIO.PRBS_DOWN_SAMPLING_RATIO", signed=False, parent=self)

class _Mcc_PidConfig(Register):
    ADDRESS = 0x4800A600
    WIDTH = 4
    DEFAULT = 0x00008550

    def __init__(self, parent):
        self._PARENT = parent

        self.KEEP_POS_TARGET = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "MCC.PID_CONFIG.KEEP_POS_TARGET", signed=False, parent=self)
        self.CURRENT_NORM_P  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "MCC.PID_CONFIG.CURRENT_NORM_P", signed=False, parent=self)
        self.CURRENT_NORM_I  = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "MCC.PID_CONFIG.CURRENT_NORM_I", signed=False, parent=self)
        self.VELOCITY_NORM_P = Field(self.ADDRESS, self.WIDTH, 0x00000030,  4, "MCC.PID_CONFIG.VELOCITY_NORM_P", signed=False, parent=self)
        self.VELOCITY_NORM_I = Field(self.ADDRESS, self.WIDTH, 0x000000C0,  6, "MCC.PID_CONFIG.VELOCITY_NORM_I", signed=False, parent=self)
        self.POSITION_NORM_P = Field(self.ADDRESS, self.WIDTH, 0x00000300,  8, "MCC.PID_CONFIG.POSITION_NORM_P", signed=False, parent=self)
        self.POSITION_NORM_I = Field(self.ADDRESS, self.WIDTH, 0x00000C00, 10, "MCC.PID_CONFIG.POSITION_NORM_I", signed=False, parent=self)
        self.VEL_SCALE       = Field(self.ADDRESS, self.WIDTH, 0x0000F000, 12, "MCC.PID_CONFIG.VEL_SCALE", signed=False, parent=self)
        self.POS_SMPL        = Field(self.ADDRESS, self.WIDTH, 0x007F0000, 16, "MCC.PID_CONFIG.POS_SMPL", signed=False, parent=self)
        self.VEL_SMPL        = Field(self.ADDRESS, self.WIDTH, 0x7F000000, 24, "MCC.PID_CONFIG.VEL_SMPL", signed=False, parent=self)

class _Mcc_PidFluxCoeff(Register):
    ADDRESS = 0x4800A604
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.I = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PID_FLUX_COEFF.I", signed=True, parent=self)
        self.P = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.PID_FLUX_COEFF.P", signed=True, parent=self)

class _Mcc_PidTorqueCoeff(Register):
    ADDRESS = 0x4800A608
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.I = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PID_TORQUE_COEFF.I", signed=True, parent=self)
        self.P = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.PID_TORQUE_COEFF.P", signed=True, parent=self)

class _Mcc_PidFieldweakCoeff(Register):
    ADDRESS = 0x4800A60C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.I = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PID_FIELDWEAK_COEFF.I", signed=True, parent=self)
        self.P = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.PID_FIELDWEAK_COEFF.P", signed=True, parent=self)

class _Mcc_PidUSMax(Register):
    ADDRESS = 0x4800A610
    WIDTH = 4
    DEFAULT = 0x00007FFF

    def __init__(self, parent):
        self._PARENT = parent

        self.U_S_MAX = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PID_U_S_MAX.U_S_MAX", signed=False, parent=self)

class _Mcc_PidVelocityCoeff(Register):
    ADDRESS = 0x4800A614
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.I = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PID_VELOCITY_COEFF.I", signed=True, parent=self)
        self.P = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.PID_VELOCITY_COEFF.P", signed=True, parent=self)

class _Mcc_PidPositionCoeff(Register):
    ADDRESS = 0x4800A618
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.I = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PID_POSITION_COEFF.I", signed=True, parent=self)
        self.P = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.PID_POSITION_COEFF.P", signed=True, parent=self)

class _Mcc_PidPositionTolerance(Register):
    ADDRESS = 0x4800A61C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_POSITION_TOLERANCE = Field(self.ADDRESS, self.WIDTH, 0x7FFFFFFF,  0, "MCC.PID_POSITION_TOLERANCE.PID_POSITION_TOLERANCE", signed=False, parent=self)

class _Mcc_PidPositionToleranceDelay(Register):
    ADDRESS = 0x4800A620
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_POSITION_TOLERANCE_DELAY = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PID_POSITION_TOLERANCE_DELAY.PID_POSITION_TOLERANCE_DELAY", signed=False, parent=self)

class _Mcc_PidUqUdLimits(Register):
    ADDRESS = 0x4800A624
    WIDTH = 4
    DEFAULT = 0x00005A81

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_UQ_UD_LIMITS = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PID_UQ_UD_LIMITS.PID_UQ_UD_LIMITS", signed=False, parent=self)

class _Mcc_PidTorqueFluxLimits(Register):
    ADDRESS = 0x4800A628
    WIDTH = 4
    DEFAULT = 0x7FFF7FFF

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_FLUX_LIMIT   = Field(self.ADDRESS, self.WIDTH, 0x00007FFF,  0, "MCC.PID_TORQUE_FLUX_LIMITS.PID_FLUX_LIMIT", signed=False, parent=self)
        self.PID_TORQUE_LIMIT = Field(self.ADDRESS, self.WIDTH, 0x7FFF0000, 16, "MCC.PID_TORQUE_FLUX_LIMITS.PID_TORQUE_LIMIT", signed=False, parent=self)

class _Mcc_PidVelocityLimit(Register):
    ADDRESS = 0x4800A62C
    WIDTH = 4
    DEFAULT = 0x7FFFFFFF

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_VELOCITY_LIMIT = Field(self.ADDRESS, self.WIDTH, 0x7FFFFFFF,  0, "MCC.PID_VELOCITY_LIMIT.PID_VELOCITY_LIMIT", signed=False, parent=self)

class _Mcc_PidPositionLimitLow(Register):
    ADDRESS = 0x4800A630
    WIDTH = 4
    DEFAULT = 0x80000001

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_POSITION_LIMIT_LOW = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PID_POSITION_LIMIT_LOW.PID_POSITION_LIMIT_LOW", signed=True, parent=self)

class _Mcc_PidPositionLimitHigh(Register):
    ADDRESS = 0x4800A634
    WIDTH = 4
    DEFAULT = 0x7FFFFFFF

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_POSITION_LIMIT_HIGH = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PID_POSITION_LIMIT_HIGH.PID_POSITION_LIMIT_HIGH", signed=True, parent=self)

class _Mcc_PidTorqueFluxTarget(Register):
    ADDRESS = 0x4800A638
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_FLUX_TARGET   = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PID_TORQUE_FLUX_TARGET.PID_FLUX_TARGET", signed=True, parent=self)
        self.PID_TORQUE_TARGET = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.PID_TORQUE_FLUX_TARGET.PID_TORQUE_TARGET", signed=True, parent=self)

class _Mcc_PidTorqueFluxOffset(Register):
    ADDRESS = 0x4800A63C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_FLUX_OFFSET   = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PID_TORQUE_FLUX_OFFSET.PID_FLUX_OFFSET", signed=True, parent=self)
        self.PID_TORQUE_OFFSET = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.PID_TORQUE_FLUX_OFFSET.PID_TORQUE_OFFSET", signed=True, parent=self)

class _Mcc_PidVelocityTarget(Register):
    ADDRESS = 0x4800A640
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_VELOCITY_TARGET = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PID_VELOCITY_TARGET.PID_VELOCITY_TARGET", signed=True, parent=self)

class _Mcc_PidVelocityOffset(Register):
    ADDRESS = 0x4800A644
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_VELOCITY_OFFSET = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PID_VELOCITY_OFFSET.PID_VELOCITY_OFFSET", signed=True, parent=self)

class _Mcc_PidPositionTarget(Register):
    ADDRESS = 0x4800A648
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_POSITION_TARGET = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PID_POSITION_TARGET.PID_POSITION_TARGET", signed=True, parent=self)

class _Mcc_PidTorqueFluxActual(Register):
    ADDRESS = 0x4800A64C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_FLUX_ACTUAL   = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL", signed=True, parent=self)
        self.PID_TORQUE_ACTUAL = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL", signed=True, parent=self)

class _Mcc_PidVelocityActual(Register):
    ADDRESS = 0x4800A650
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_VELOCITY_ACTUAL = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL", signed=True, parent=self)

class _Mcc_PidPositionActual(Register):
    ADDRESS = 0x4800A654
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_POSITION_ACTUAL = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PID_POSITION_ACTUAL.PID_POSITION_ACTUAL", signed=True, parent=self)

class _Mcc_PidPositionActualOffset(Register):
    ADDRESS = 0x4800A658
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_POSITION_ACTUAL_OFFSET = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PID_POSITION_ACTUAL_OFFSET.PID_POSITION_ACTUAL_OFFSET", signed=True, parent=self)

class _Mcc_PidTorqueError(Register):
    ADDRESS = 0x4800A65C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_TORQUE_ERROR = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PID_TORQUE_ERROR.PID_TORQUE_ERROR", signed=True, parent=self)

class _Mcc_PidFluxError(Register):
    ADDRESS = 0x4800A660
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_FLUX_ERROR = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PID_FLUX_ERROR.PID_FLUX_ERROR", signed=True, parent=self)

class _Mcc_PidVelocityError(Register):
    ADDRESS = 0x4800A664
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_VELOCITY_ERROR = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PID_VELOCITY_ERROR.PID_VELOCITY_ERROR", signed=True, parent=self)

class _Mcc_PidPositionError(Register):
    ADDRESS = 0x4800A668
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_POSITION_ERROR = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PID_POSITION_ERROR.PID_POSITION_ERROR", signed=True, parent=self)

class _Mcc_PidTorqueIntegrator(Register):
    ADDRESS = 0x4800A66C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_TORQUE_INTEGRATOR = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PID_TORQUE_INTEGRATOR.PID_TORQUE_INTEGRATOR", signed=True, parent=self)

class _Mcc_PidFluxIntegrator(Register):
    ADDRESS = 0x4800A670
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_FLUX_INTEGRATOR = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PID_FLUX_INTEGRATOR.PID_FLUX_INTEGRATOR", signed=True, parent=self)

class _Mcc_PidVelocityIntegrator(Register):
    ADDRESS = 0x4800A674
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_VELOCITY_INTEGRATOR = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PID_VELOCITY_INTEGRATOR.PID_VELOCITY_INTEGRATOR", signed=True, parent=self)

class _Mcc_PidPositionIntegrator(Register):
    ADDRESS = 0x4800A678
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_POSITION_INTEGRATOR = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PID_POSITION_INTEGRATOR.PID_POSITION_INTEGRATOR", signed=True, parent=self)

class _Mcc_PidinTorqueFluxTarget(Register):
    ADDRESS = 0x4800A680
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PIDIN_FLUX_TARGET   = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PIDIN_TORQUE_FLUX_TARGET.PIDIN_FLUX_TARGET", signed=True, parent=self)
        self.PIDIN_TORQUE_TARGET = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.PIDIN_TORQUE_FLUX_TARGET.PIDIN_TORQUE_TARGET", signed=True, parent=self)

class _Mcc_PidinVelocityTarget(Register):
    ADDRESS = 0x4800A684
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PIDIN_VELOCITY_TARGET = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PIDIN_VELOCITY_TARGET.PIDIN_VELOCITY_TARGET", signed=True, parent=self)

class _Mcc_PidinPositionTarget(Register):
    ADDRESS = 0x4800A688
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PIDIN_POSITION_TARGET = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PIDIN_POSITION_TARGET.PIDIN_POSITION_TARGET", signed=True, parent=self)

class _Mcc_PidinTorqueFluxTargetLimited(Register):
    ADDRESS = 0x4800A68C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PIDIN_FLUX_TARGET_LIMITED   = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PIDIN_TORQUE_FLUX_TARGET_LIMITED.PIDIN_FLUX_TARGET_LIMITED", signed=True, parent=self)
        self.PIDIN_TORQUE_TARGET_LIMITED = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.PIDIN_TORQUE_FLUX_TARGET_LIMITED.PIDIN_TORQUE_TARGET_LIMITED", signed=True, parent=self)

class _Mcc_PidinVelocityTargetLimited(Register):
    ADDRESS = 0x4800A690
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PIDIN_VELOCITY_TARGET_LIMITED = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PIDIN_VELOCITY_TARGET_LIMITED.PIDIN_VELOCITY_TARGET_LIMITED", signed=True, parent=self)

class _Mcc_PidinPositionTargetLimited(Register):
    ADDRESS = 0x4800A694
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PIDIN_POSITION_TARGET_LIMITED = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.PIDIN_POSITION_TARGET_LIMITED.PIDIN_POSITION_TARGET_LIMITED", signed=True, parent=self)

class _Mcc_FocIbetaIalpha(Register):
    ADDRESS = 0x4800A698
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.IALPHA = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.FOC_IBETA_IALPHA.IALPHA", signed=True, parent=self)
        self.IBETA  = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.FOC_IBETA_IALPHA.IBETA", signed=True, parent=self)

class _Mcc_FocIqId(Register):
    ADDRESS = 0x4800A69C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.ID = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.FOC_IQ_ID.ID", signed=True, parent=self)
        self.IQ = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.FOC_IQ_ID.IQ", signed=True, parent=self)

class _Mcc_FocUqUd(Register):
    ADDRESS = 0x4800A6A0
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.UD = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.FOC_UQ_UD.UD", signed=True, parent=self)
        self.UQ = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.FOC_UQ_UD.UQ", signed=True, parent=self)

class _Mcc_FocUqUdLimited(Register):
    ADDRESS = 0x4800A6A4
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.UD = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.FOC_UQ_UD_LIMITED.UD", signed=True, parent=self)
        self.UQ = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.FOC_UQ_UD_LIMITED.UQ", signed=True, parent=self)

class _Mcc_FocUbetaUalpha(Register):
    ADDRESS = 0x4800A6A8
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.UALPHA = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.FOC_UBETA_UALPHA.UALPHA", signed=True, parent=self)
        self.UBETA  = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.FOC_UBETA_UALPHA.UBETA", signed=True, parent=self)

class _Mcc_FocUwyUux(Register):
    ADDRESS = 0x4800A6AC
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.UUX = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.FOC_UWY_UUX.UUX", signed=True, parent=self)
        self.UWY = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.FOC_UWY_UUX.UWY", signed=True, parent=self)

class _Mcc_FocUv(Register):
    ADDRESS = 0x4800A6B0
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.UV = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.FOC_UV.UV", signed=True, parent=self)

class _Mcc_PwmVx2Ux1(Register):
    ADDRESS = 0x4800A6B4
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.UX1 = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PWM_VX2_UX1.UX1", signed=False, parent=self)
        self.VX2 = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.PWM_VX2_UX1.VX2", signed=False, parent=self)

class _Mcc_PwmY2Wy1(Register):
    ADDRESS = 0x4800A6B8
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.WY1 = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PWM_Y2_WY1.WY1", signed=False, parent=self)
        self.Y2  = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.PWM_Y2_WY1.Y2", signed=False, parent=self)

class _Mcc_VelocityFrq(Register):
    ADDRESS = 0x4800A6BC
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.VELOCITY_FRQ = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.VELOCITY_FRQ.VELOCITY_FRQ", signed=True, parent=self)

class _Mcc_VelocityPer(Register):
    ADDRESS = 0x4800A6C0
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.VELOCITY_PER = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.VELOCITY_PER.VELOCITY_PER", signed=True, parent=self)

class _Mcc_FocStatus(Register):
    ADDRESS = 0x4800A6C4
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.FOC_STATUS = Field(self.ADDRESS, self.WIDTH, 0x0000000F,  0, "MCC.FOC_STATUS.FOC_STATUS", signed=False, parent=self)

class _Mcc_USActualISActual(Register):
    ADDRESS = 0x4800A700
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.I_S_ACTUAL = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.U_S_ACTUAL_I_S_ACTUAL.I_S_ACTUAL", signed=False, parent=self)
        self.U_S_ACTUAL = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.U_S_ACTUAL_I_S_ACTUAL.U_S_ACTUAL", signed=False, parent=self)

class _Mcc_PMotor(Register):
    ADDRESS = 0x4800A704
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.P_MOTOR = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.P_MOTOR.P_MOTOR", signed=False, parent=self)

class _Mcc_InputsRaw(Register):
    ADDRESS = 0x4800A708
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.ENC_A       = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "MCC.INPUTS_RAW.ENC_A", signed=False, parent=self)
        self.ENC_B       = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "MCC.INPUTS_RAW.ENC_B", signed=False, parent=self)
        self.ENC_N       = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "MCC.INPUTS_RAW.ENC_N", signed=False, parent=self)
        self.HALL_U      = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "MCC.INPUTS_RAW.HALL_U", signed=False, parent=self)
        self.HALL_V      = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "MCC.INPUTS_RAW.HALL_V", signed=False, parent=self)
        self.HALL_W      = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "MCC.INPUTS_RAW.HALL_W", signed=False, parent=self)
        self.REF_SW_R    = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "MCC.INPUTS_RAW.REF_SW_R", signed=False, parent=self)
        self.REF_SW_L    = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "MCC.INPUTS_RAW.REF_SW_L", signed=False, parent=self)
        self.REF_SW_H    = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "MCC.INPUTS_RAW.REF_SW_H", signed=False, parent=self)
        self.ENI         = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "MCC.INPUTS_RAW.ENI", signed=False, parent=self)
        self.HALL_U_FILT = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "MCC.INPUTS_RAW.HALL_U_FILT", signed=False, parent=self)
        self.HALL_V_FILT = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "MCC.INPUTS_RAW.HALL_V_FILT", signed=False, parent=self)
        self.HALL_W_FILT = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "MCC.INPUTS_RAW.HALL_W_FILT", signed=False, parent=self)

class _Mcc_OutputsRaw(Register):
    ADDRESS = 0x4800A70C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PWM_UX1_L = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "MCC.OUTPUTS_RAW.PWM_UX1_L", signed=False, parent=self)
        self.PWM_UX1_H = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "MCC.OUTPUTS_RAW.PWM_UX1_H", signed=False, parent=self)
        self.PWM_VX2_L = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "MCC.OUTPUTS_RAW.PWM_VX2_L", signed=False, parent=self)
        self.PWM_VX2_H = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "MCC.OUTPUTS_RAW.PWM_VX2_H", signed=False, parent=self)
        self.PWM_WY1_L = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "MCC.OUTPUTS_RAW.PWM_WY1_L", signed=False, parent=self)
        self.PWM_WY1_H = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "MCC.OUTPUTS_RAW.PWM_WY1_H", signed=False, parent=self)
        self.PWM_Y2_L  = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "MCC.OUTPUTS_RAW.PWM_Y2_L", signed=False, parent=self)
        self.PWM_Y2_H  = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "MCC.OUTPUTS_RAW.PWM_Y2_H", signed=False, parent=self)

class _Mcc_StatusFlags(Register):
    ADDRESS = 0x4800A710
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PID_X_TARGET_LIMIT      = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "MCC.STATUS_FLAGS.PID_X_TARGET_LIMIT", signed=False, parent=self)
        self.PID_X_OUTPUT_LIMIT      = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "MCC.STATUS_FLAGS.PID_X_OUTPUT_LIMIT", signed=False, parent=self)
        self.PID_V_TARGET_LIMIT      = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "MCC.STATUS_FLAGS.PID_V_TARGET_LIMIT", signed=False, parent=self)
        self.PID_V_OUTPUT_LIMIT      = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "MCC.STATUS_FLAGS.PID_V_OUTPUT_LIMIT", signed=False, parent=self)
        self.PID_ID_TARGET_LIMIT     = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "MCC.STATUS_FLAGS.PID_ID_TARGET_LIMIT", signed=False, parent=self)
        self.PID_ID_OUTPUT_LIMIT     = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "MCC.STATUS_FLAGS.PID_ID_OUTPUT_LIMIT", signed=False, parent=self)
        self.PID_IQ_TARGET_LIMIT     = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "MCC.STATUS_FLAGS.PID_IQ_TARGET_LIMIT", signed=False, parent=self)
        self.PID_IQ_OUTPUT_LIMIT     = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "MCC.STATUS_FLAGS.PID_IQ_OUTPUT_LIMIT", signed=False, parent=self)
        self.IPARK_VOLTLIM_LIMIT_U   = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "MCC.STATUS_FLAGS.IPARK_VOLTLIM_LIMIT_U", signed=False, parent=self)
        self.PWM_SWITCH_LIMIT_ACTIVE = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "MCC.STATUS_FLAGS.PWM_SWITCH_LIMIT_ACTIVE", signed=False, parent=self)
        self.HALL_ERROR              = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "MCC.STATUS_FLAGS.HALL_ERROR", signed=False, parent=self)
        self.POSITION_TRACKING_ERROR = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "MCC.STATUS_FLAGS.POSITION_TRACKING_ERROR", signed=False, parent=self)
        self.VELOCITY_TRACKING_ERROR = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "MCC.STATUS_FLAGS.VELOCITY_TRACKING_ERROR", signed=False, parent=self)
        self.PID_FW_OUTPUT_LIMIT     = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "MCC.STATUS_FLAGS.PID_FW_OUTPUT_LIMIT", signed=False, parent=self)
        self.WATCHDOG                = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "MCC.STATUS_FLAGS.WATCHDOG", signed=False, parent=self)
        self.SHORT                   = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "MCC.STATUS_FLAGS.SHORT", signed=False, parent=self)
        self.REF_SW_L                = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "MCC.STATUS_FLAGS.REF_SW_L", signed=False, parent=self)
        self.REF_SW_R                = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "MCC.STATUS_FLAGS.REF_SW_R", signed=False, parent=self)
        self.REF_SW_H                = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "MCC.STATUS_FLAGS.REF_SW_H", signed=False, parent=self)
        self.POSITION_REACHED        = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "MCC.STATUS_FLAGS.POSITION_REACHED", signed=False, parent=self)
        self.ADC_I_CLIPPED           = Field(self.ADDRESS, self.WIDTH, 0x04000000, 26, "MCC.STATUS_FLAGS.ADC_I_CLIPPED", signed=False, parent=self)
        self.ENC_N                   = Field(self.ADDRESS, self.WIDTH, 0x10000000, 28, "MCC.STATUS_FLAGS.ENC_N", signed=False, parent=self)
        self.ENI                     = Field(self.ADDRESS, self.WIDTH, 0x80000000, 31, "MCC.STATUS_FLAGS.ENI", signed=False, parent=self)

class _Mcc_StatusMask(Register):
    ADDRESS = 0x4800A714
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.STATUS_MASK = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.STATUS_MASK.STATUS_MASK", signed=False, parent=self)

class _Mcc_FlexCompConf(Register):
    ADDRESS = 0x4800A780
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.START_SELECT   = Field(self.ADDRESS, self.WIDTH, 0x0000000F,  0, "MCC.FLEX_COMP_CONF.START_SELECT", signed=False, parent=self)
        self.START_DEGLITCH = Field(self.ADDRESS, self.WIDTH, 0x000000F0,  4, "MCC.FLEX_COMP_CONF.START_DEGLITCH", signed=False, parent=self)
        self.START_EDGE     = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "MCC.FLEX_COMP_CONF.START_EDGE", signed=False, parent=self)
        self.END_SELECT     = Field(self.ADDRESS, self.WIDTH, 0x00001E00,  9, "MCC.FLEX_COMP_CONF.END_SELECT", signed=False, parent=self)
        self.END_DEGLITCH   = Field(self.ADDRESS, self.WIDTH, 0x0001E000, 13, "MCC.FLEX_COMP_CONF.END_DEGLITCH", signed=False, parent=self)
        self.END_EDGE       = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "MCC.FLEX_COMP_CONF.END_EDGE", signed=False, parent=self)
        self.SINGLE         = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "MCC.FLEX_COMP_CONF.SINGLE", signed=False, parent=self)
        self.CONTINUOUS     = Field(self.ADDRESS, self.WIDTH, 0x02000000, 25, "MCC.FLEX_COMP_CONF.CONTINUOUS", signed=False, parent=self)
        self.DONE_U         = Field(self.ADDRESS, self.WIDTH, 0x10000000, 28, "MCC.FLEX_COMP_CONF.DONE_U", signed=False, parent=self)
        self.DONE_V         = Field(self.ADDRESS, self.WIDTH, 0x20000000, 29, "MCC.FLEX_COMP_CONF.DONE_V", signed=False, parent=self)
        self.DONE_W         = Field(self.ADDRESS, self.WIDTH, 0x40000000, 30, "MCC.FLEX_COMP_CONF.DONE_W", signed=False, parent=self)
        self.DONE_Y2        = Field(self.ADDRESS, self.WIDTH, 0x80000000, 31, "MCC.FLEX_COMP_CONF.DONE_Y2", signed=False, parent=self)

class _Mcc_FlexCompResultVU(Register):
    ADDRESS = 0x4800A784
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.U = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.FLEX_COMP_RESULT_V_U.U", signed=False, parent=self)
        self.V = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.FLEX_COMP_RESULT_V_U.V", signed=False, parent=self)

class _Mcc_FlexCompResultY2W(Register):
    ADDRESS = 0x4800A788
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.W  = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.FLEX_COMP_RESULT_Y2_W.W", signed=False, parent=self)
        self.Y2 = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.FLEX_COMP_RESULT_Y2_W.Y2", signed=False, parent=self)

class _Mcc_GdrvHw(Register):
    ADDRESS = 0x4800A78C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.BRIDGE_ENABLE_U  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "MCC.GDRV_HW.BRIDGE_ENABLE_U", signed=False, parent=self)
        self.BRIDGE_ENABLE_V  = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "MCC.GDRV_HW.BRIDGE_ENABLE_V", signed=False, parent=self)
        self.BRIDGE_ENABLE_W  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "MCC.GDRV_HW.BRIDGE_ENABLE_W", signed=False, parent=self)
        self.BRIDGE_ENABLE_Y2 = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "MCC.GDRV_HW.BRIDGE_ENABLE_Y2", signed=False, parent=self)
        self.LS_OCP_CMP_EN    = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "MCC.GDRV_HW.LS_OCP_CMP_EN", signed=False, parent=self)
        self.HS_OCP_CMP_EN    = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "MCC.GDRV_HW.HS_OCP_CMP_EN", signed=False, parent=self)
        self.VDRV_UVLO_CMP_EN = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "MCC.GDRV_HW.VDRV_UVLO_CMP_EN", signed=False, parent=self)
        self.VS_UVLO_CMP_EN   = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "MCC.GDRV_HW.VS_UVLO_CMP_EN", signed=False, parent=self)
        self.BST_ILIM_MAX     = Field(self.ADDRESS, self.WIDTH, 0x00000700,  8, "MCC.GDRV_HW.BST_ILIM_MAX", signed=False, parent=self)
        self.BST_SW_CP_EN     = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "MCC.GDRV_HW.BST_SW_CP_EN", signed=False, parent=self)
        self.DISCHARGE_BST    = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "MCC.GDRV_HW.DISCHARGE_BST", signed=False, parent=self)
        self.PHASE_DIV_GAIN   = Field(self.ADDRESS, self.WIDTH, 0x00030000, 16, "MCC.GDRV_HW.PHASE_DIV_GAIN", signed=False, parent=self)
        self.PHASE_DIV_EN_UVW = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "MCC.GDRV_HW.PHASE_DIV_EN_UVW", signed=False, parent=self)
        self.PHASE_DIV_EN_Y2  = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "MCC.GDRV_HW.PHASE_DIV_EN_Y2", signed=False, parent=self)
        self.PHASE_CMP_EN_UVW = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "MCC.GDRV_HW.PHASE_CMP_EN_UVW", signed=False, parent=self)
        self.PHASE_CMP_EN_Y2  = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "MCC.GDRV_HW.PHASE_CMP_EN_Y2", signed=False, parent=self)
        self.CHARGEPUMP_EN    = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "MCC.GDRV_HW.CHARGEPUMP_EN", signed=False, parent=self)
        self.BIAS_EN          = Field(self.ADDRESS, self.WIDTH, 0x02000000, 25, "MCC.GDRV_HW.BIAS_EN", signed=False, parent=self)
        self.HS_AS_LS_Y2      = Field(self.ADDRESS, self.WIDTH, 0x10000000, 28, "MCC.GDRV_HW.HS_AS_LS_Y2", signed=False, parent=self)
        self.HIGHZ_ON_IDLE    = Field(self.ADDRESS, self.WIDTH, 0x20000000, 29, "MCC.GDRV_HW.HIGHZ_ON_IDLE", signed=False, parent=self)

class _Mcc_GdrvCfg(Register):
    ADDRESS = 0x4800A790
    WIDTH = 4
    DEFAULT = 0x00030000

    def __init__(self, parent):
        self._PARENT = parent

        self.IGATE_SINK_UVW    = Field(self.ADDRESS, self.WIDTH, 0x0000000F,  0, "MCC.GDRV_CFG.IGATE_SINK_UVW", signed=False, parent=self)
        self.IGATE_SOURCE_UVW  = Field(self.ADDRESS, self.WIDTH, 0x000000F0,  4, "MCC.GDRV_CFG.IGATE_SOURCE_UVW", signed=False, parent=self)
        self.IGATE_SINK_Y2     = Field(self.ADDRESS, self.WIDTH, 0x00000F00,  8, "MCC.GDRV_CFG.IGATE_SINK_Y2", signed=False, parent=self)
        self.IGATE_SOURCE_Y2   = Field(self.ADDRESS, self.WIDTH, 0x0000F000, 12, "MCC.GDRV_CFG.IGATE_SOURCE_Y2", signed=False, parent=self)
        self.ADAPTIVE_MODE_UVW = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "MCC.GDRV_CFG.ADAPTIVE_MODE_UVW", signed=False, parent=self)
        self.ADAPTIVE_MODE_Y2  = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "MCC.GDRV_CFG.ADAPTIVE_MODE_Y2", signed=False, parent=self)
        self.VS_UVLO_LVL       = Field(self.ADDRESS, self.WIDTH, 0x00F00000, 20, "MCC.GDRV_CFG.VS_UVLO_LVL", signed=False, parent=self)
        self.EXT_LS_POL        = Field(self.ADDRESS, self.WIDTH, 0x10000000, 28, "MCC.GDRV_CFG.EXT_LS_POL", signed=False, parent=self)
        self.EXT_HS_POL        = Field(self.ADDRESS, self.WIDTH, 0x20000000, 29, "MCC.GDRV_CFG.EXT_HS_POL", signed=False, parent=self)
        self.EXT_MODE          = Field(self.ADDRESS, self.WIDTH, 0x40000000, 30, "MCC.GDRV_CFG.EXT_MODE", signed=False, parent=self)

class _Mcc_GdrvTiming(Register):
    ADDRESS = 0x4800A7A4
    WIDTH = 4
    DEFAULT = 0xFFFFFFFF

    def __init__(self, parent):
        self._PARENT = parent

        self.T_DRIVE_SINK_UVW   = Field(self.ADDRESS, self.WIDTH, 0x000000FF,  0, "MCC.GDRV_TIMING.T_DRIVE_SINK_UVW", signed=False, parent=self)
        self.T_DRIVE_SOURCE_UVW = Field(self.ADDRESS, self.WIDTH, 0x0000FF00,  8, "MCC.GDRV_TIMING.T_DRIVE_SOURCE_UVW", signed=False, parent=self)
        self.T_DRIVE_SINK_Y2    = Field(self.ADDRESS, self.WIDTH, 0x00FF0000, 16, "MCC.GDRV_TIMING.T_DRIVE_SINK_Y2", signed=False, parent=self)
        self.T_DRIVE_SOURCE_Y2  = Field(self.ADDRESS, self.WIDTH, 0xFF000000, 24, "MCC.GDRV_TIMING.T_DRIVE_SOURCE_Y2", signed=False, parent=self)

class _Mcc_GdrvBbm(Register):
    ADDRESS = 0x4800A7A8
    WIDTH = 4
    DEFAULT = 0x14141414

    def __init__(self, parent):
        self._PARENT = parent

        self.BBM_L_UVW = Field(self.ADDRESS, self.WIDTH, 0x000000FF,  0, "MCC.GDRV_BBM.BBM_L_UVW", signed=False, parent=self)
        self.BBM_H_UVW = Field(self.ADDRESS, self.WIDTH, 0x0000FF00,  8, "MCC.GDRV_BBM.BBM_H_UVW", signed=False, parent=self)
        self.BBM_L_Y2  = Field(self.ADDRESS, self.WIDTH, 0x00FF0000, 16, "MCC.GDRV_BBM.BBM_L_Y2", signed=False, parent=self)
        self.BBM_H_Y2  = Field(self.ADDRESS, self.WIDTH, 0xFF000000, 24, "MCC.GDRV_BBM.BBM_H_Y2", signed=False, parent=self)

class _Mcc_GdrvProt(Register):
    ADDRESS = 0x4800A7AC
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.VGS_DEGLITCH_UVW  = Field(self.ADDRESS, self.WIDTH, 0x00000007,  0, "MCC.GDRV_PROT.VGS_DEGLITCH_UVW", signed=False, parent=self)
        self.VGS_BLANKING_UVW  = Field(self.ADDRESS, self.WIDTH, 0x00000030,  4, "MCC.GDRV_PROT.VGS_BLANKING_UVW", signed=False, parent=self)
        self.VGS_DEGLITCH_Y2   = Field(self.ADDRESS, self.WIDTH, 0x00000700,  8, "MCC.GDRV_PROT.VGS_DEGLITCH_Y2", signed=False, parent=self)
        self.VGS_BLANKING_Y2   = Field(self.ADDRESS, self.WIDTH, 0x00003000, 12, "MCC.GDRV_PROT.VGS_BLANKING_Y2", signed=False, parent=self)
        self.LS_RETRIES_UVW    = Field(self.ADDRESS, self.WIDTH, 0x00030000, 16, "MCC.GDRV_PROT.LS_RETRIES_UVW", signed=False, parent=self)
        self.HS_RETRIES_UVW    = Field(self.ADDRESS, self.WIDTH, 0x000C0000, 18, "MCC.GDRV_PROT.HS_RETRIES_UVW", signed=False, parent=self)
        self.LS_RETRIES_Y2     = Field(self.ADDRESS, self.WIDTH, 0x00300000, 20, "MCC.GDRV_PROT.LS_RETRIES_Y2", signed=False, parent=self)
        self.HS_RETRIES_Y2     = Field(self.ADDRESS, self.WIDTH, 0x00C00000, 22, "MCC.GDRV_PROT.HS_RETRIES_Y2", signed=False, parent=self)
        self.TERM_PWM_ON_SHORT = Field(self.ADDRESS, self.WIDTH, 0x10000000, 28, "MCC.GDRV_PROT.TERM_PWM_ON_SHORT", signed=False, parent=self)

class _Mcc_GdrvOcpUvw(Register):
    ADDRESS = 0x4800A7B0
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.LS_OCP_DEGLITCH_UVW = Field(self.ADDRESS, self.WIDTH, 0x00000007,  0, "MCC.GDRV_OCP_UVW.LS_OCP_DEGLITCH_UVW", signed=False, parent=self)
        self.LS_OCP_BLANKING_UVW = Field(self.ADDRESS, self.WIDTH, 0x00000070,  4, "MCC.GDRV_OCP_UVW.LS_OCP_BLANKING_UVW", signed=False, parent=self)
        self.LS_OCP_THRES_UVW    = Field(self.ADDRESS, self.WIDTH, 0x00000F00,  8, "MCC.GDRV_OCP_UVW.LS_OCP_THRES_UVW", signed=False, parent=self)
        self.LS_OCP_USE_VDS_UVW  = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "MCC.GDRV_OCP_UVW.LS_OCP_USE_VDS_UVW", signed=False, parent=self)
        self.HS_OCP_DEGLITCH_UVW = Field(self.ADDRESS, self.WIDTH, 0x00070000, 16, "MCC.GDRV_OCP_UVW.HS_OCP_DEGLITCH_UVW", signed=False, parent=self)
        self.HS_OCP_BLANKING_UVW = Field(self.ADDRESS, self.WIDTH, 0x00700000, 20, "MCC.GDRV_OCP_UVW.HS_OCP_BLANKING_UVW", signed=False, parent=self)
        self.HS_OCP_THRES_UVW    = Field(self.ADDRESS, self.WIDTH, 0x0F000000, 24, "MCC.GDRV_OCP_UVW.HS_OCP_THRES_UVW", signed=False, parent=self)

class _Mcc_GdrvOcpY2(Register):
    ADDRESS = 0x4800A7B4
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.LS_OCP_DEGLITCH_Y2 = Field(self.ADDRESS, self.WIDTH, 0x00000007,  0, "MCC.GDRV_OCP_Y2.LS_OCP_DEGLITCH_Y2", signed=False, parent=self)
        self.LS_OCP_BLANKING_Y2 = Field(self.ADDRESS, self.WIDTH, 0x00000070,  4, "MCC.GDRV_OCP_Y2.LS_OCP_BLANKING_Y2", signed=False, parent=self)
        self.LS_OCP_THRES_Y2    = Field(self.ADDRESS, self.WIDTH, 0x00000F00,  8, "MCC.GDRV_OCP_Y2.LS_OCP_THRES_Y2", signed=False, parent=self)
        self.LS_OCP_USE_VDS_Y2  = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "MCC.GDRV_OCP_Y2.LS_OCP_USE_VDS_Y2", signed=False, parent=self)
        self.HS_OCP_DEGLITCH_Y2 = Field(self.ADDRESS, self.WIDTH, 0x00070000, 16, "MCC.GDRV_OCP_Y2.HS_OCP_DEGLITCH_Y2", signed=False, parent=self)
        self.HS_OCP_BLANKING_Y2 = Field(self.ADDRESS, self.WIDTH, 0x00700000, 20, "MCC.GDRV_OCP_Y2.HS_OCP_BLANKING_Y2", signed=False, parent=self)
        self.HS_OCP_THRES_Y2    = Field(self.ADDRESS, self.WIDTH, 0x0F000000, 24, "MCC.GDRV_OCP_Y2.HS_OCP_THRES_Y2", signed=False, parent=self)

class _Mcc_GdrvProtEn(Register):
    ADDRESS = 0x4800A7B8
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.LS_SHORT_PROT_U          = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "MCC.GDRV_PROT_EN.LS_SHORT_PROT_U", signed=False, parent=self)
        self.LS_SHORT_PROT_V          = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "MCC.GDRV_PROT_EN.LS_SHORT_PROT_V", signed=False, parent=self)
        self.LS_SHORT_PROT_W          = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "MCC.GDRV_PROT_EN.LS_SHORT_PROT_W", signed=False, parent=self)
        self.LS_SHORT_PROT_Y2         = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "MCC.GDRV_PROT_EN.LS_SHORT_PROT_Y2", signed=False, parent=self)
        self.LS_VGS_OFF_SHORT_PROT_U  = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "MCC.GDRV_PROT_EN.LS_VGS_OFF_SHORT_PROT_U", signed=False, parent=self)
        self.LS_VGS_OFF_SHORT_PROT_V  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "MCC.GDRV_PROT_EN.LS_VGS_OFF_SHORT_PROT_V", signed=False, parent=self)
        self.LS_VGS_OFF_SHORT_PROT_W  = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "MCC.GDRV_PROT_EN.LS_VGS_OFF_SHORT_PROT_W", signed=False, parent=self)
        self.LS_VGS_OFF_SHORT_PROT_Y2 = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "MCC.GDRV_PROT_EN.LS_VGS_OFF_SHORT_PROT_Y2", signed=False, parent=self)
        self.LS_VGS_ON_SHORT_PROT_U   = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "MCC.GDRV_PROT_EN.LS_VGS_ON_SHORT_PROT_U", signed=False, parent=self)
        self.LS_VGS_ON_SHORT_PROT_V   = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "MCC.GDRV_PROT_EN.LS_VGS_ON_SHORT_PROT_V", signed=False, parent=self)
        self.LS_VGS_ON_SHORT_PROT_W   = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "MCC.GDRV_PROT_EN.LS_VGS_ON_SHORT_PROT_W", signed=False, parent=self)
        self.LS_VGS_ON_SHORT_PROT_Y2  = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "MCC.GDRV_PROT_EN.LS_VGS_ON_SHORT_PROT_Y2", signed=False, parent=self)
        self.BST_UVLO_PROT_U          = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "MCC.GDRV_PROT_EN.BST_UVLO_PROT_U", signed=False, parent=self)
        self.BST_UVLO_PROT_V          = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "MCC.GDRV_PROT_EN.BST_UVLO_PROT_V", signed=False, parent=self)
        self.BST_UVLO_PROT_W          = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "MCC.GDRV_PROT_EN.BST_UVLO_PROT_W", signed=False, parent=self)
        self.BST_UVLO_PROT_Y2         = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "MCC.GDRV_PROT_EN.BST_UVLO_PROT_Y2", signed=False, parent=self)
        self.HS_SHORT_PROT_U          = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "MCC.GDRV_PROT_EN.HS_SHORT_PROT_U", signed=False, parent=self)
        self.HS_SHORT_PROT_V          = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "MCC.GDRV_PROT_EN.HS_SHORT_PROT_V", signed=False, parent=self)
        self.HS_SHORT_PROT_W          = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "MCC.GDRV_PROT_EN.HS_SHORT_PROT_W", signed=False, parent=self)
        self.HS_SHORT_PROT_Y2         = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "MCC.GDRV_PROT_EN.HS_SHORT_PROT_Y2", signed=False, parent=self)
        self.HS_VGS_OFF_SHORT_PROT_U  = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "MCC.GDRV_PROT_EN.HS_VGS_OFF_SHORT_PROT_U", signed=False, parent=self)
        self.HS_VGS_OFF_SHORT_PROT_V  = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "MCC.GDRV_PROT_EN.HS_VGS_OFF_SHORT_PROT_V", signed=False, parent=self)
        self.HS_VGS_OFF_SHORT_PROT_W  = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "MCC.GDRV_PROT_EN.HS_VGS_OFF_SHORT_PROT_W", signed=False, parent=self)
        self.HS_VGS_OFF_SHORT_PROT_Y2 = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "MCC.GDRV_PROT_EN.HS_VGS_OFF_SHORT_PROT_Y2", signed=False, parent=self)
        self.HS_VGS_ON_SHORT_PROT_U   = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "MCC.GDRV_PROT_EN.HS_VGS_ON_SHORT_PROT_U", signed=False, parent=self)
        self.HS_VGS_ON_SHORT_PROT_V   = Field(self.ADDRESS, self.WIDTH, 0x02000000, 25, "MCC.GDRV_PROT_EN.HS_VGS_ON_SHORT_PROT_V", signed=False, parent=self)
        self.HS_VGS_ON_SHORT_PROT_W   = Field(self.ADDRESS, self.WIDTH, 0x04000000, 26, "MCC.GDRV_PROT_EN.HS_VGS_ON_SHORT_PROT_W", signed=False, parent=self)
        self.HS_VGS_ON_SHORT_PROT_Y2  = Field(self.ADDRESS, self.WIDTH, 0x08000000, 27, "MCC.GDRV_PROT_EN.HS_VGS_ON_SHORT_PROT_Y2", signed=False, parent=self)
        self.VDRV_UVLO_PROT           = Field(self.ADDRESS, self.WIDTH, 0x20000000, 29, "MCC.GDRV_PROT_EN.VDRV_UVLO_PROT", signed=False, parent=self)
        self.VS_UVLO_PROT             = Field(self.ADDRESS, self.WIDTH, 0x80000000, 31, "MCC.GDRV_PROT_EN.VS_UVLO_PROT", signed=False, parent=self)

class _Mcc_GdrvStatusEn(Register):
    ADDRESS = 0x4800A7BC
    WIDTH = 4
    DEFAULT = 0xEFFFFFFF

    def __init__(self, parent):
        self._PARENT = parent

        self.LS_SHORT_EN_U          = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "MCC.GDRV_STATUS_EN.LS_SHORT_EN_U", signed=False, parent=self)
        self.LS_SHORT_EN_V          = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "MCC.GDRV_STATUS_EN.LS_SHORT_EN_V", signed=False, parent=self)
        self.LS_SHORT_EN_W          = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "MCC.GDRV_STATUS_EN.LS_SHORT_EN_W", signed=False, parent=self)
        self.LS_SHORT_EN_Y2         = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "MCC.GDRV_STATUS_EN.LS_SHORT_EN_Y2", signed=False, parent=self)
        self.LS_VGS_OFF_SHORT_EN_U  = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "MCC.GDRV_STATUS_EN.LS_VGS_OFF_SHORT_EN_U", signed=False, parent=self)
        self.LS_VGS_OFF_SHORT_EN_V  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "MCC.GDRV_STATUS_EN.LS_VGS_OFF_SHORT_EN_V", signed=False, parent=self)
        self.LS_VGS_OFF_SHORT_EN_W  = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "MCC.GDRV_STATUS_EN.LS_VGS_OFF_SHORT_EN_W", signed=False, parent=self)
        self.LS_VGS_OFF_SHORT_EN_Y2 = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "MCC.GDRV_STATUS_EN.LS_VGS_OFF_SHORT_EN_Y2", signed=False, parent=self)
        self.LS_VGS_ON_SHORT_EN_U   = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "MCC.GDRV_STATUS_EN.LS_VGS_ON_SHORT_EN_U", signed=False, parent=self)
        self.LS_VGS_ON_SHORT_EN_V   = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "MCC.GDRV_STATUS_EN.LS_VGS_ON_SHORT_EN_V", signed=False, parent=self)
        self.LS_VGS_ON_SHORT_EN_W   = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "MCC.GDRV_STATUS_EN.LS_VGS_ON_SHORT_EN_W", signed=False, parent=self)
        self.LS_VGS_ON_SHORT_EN_Y2  = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "MCC.GDRV_STATUS_EN.LS_VGS_ON_SHORT_EN_Y2", signed=False, parent=self)
        self.BST_UVLO_EN_U          = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "MCC.GDRV_STATUS_EN.BST_UVLO_EN_U", signed=False, parent=self)
        self.BST_UVLO_EN_V          = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "MCC.GDRV_STATUS_EN.BST_UVLO_EN_V", signed=False, parent=self)
        self.BST_UVLO_EN_W          = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "MCC.GDRV_STATUS_EN.BST_UVLO_EN_W", signed=False, parent=self)
        self.BST_UVLO_EN_Y2         = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "MCC.GDRV_STATUS_EN.BST_UVLO_EN_Y2", signed=False, parent=self)
        self.HS_SHORT_EN_U          = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "MCC.GDRV_STATUS_EN.HS_SHORT_EN_U", signed=False, parent=self)
        self.HS_SHORT_EN_V          = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "MCC.GDRV_STATUS_EN.HS_SHORT_EN_V", signed=False, parent=self)
        self.HS_SHORT_EN_W          = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "MCC.GDRV_STATUS_EN.HS_SHORT_EN_W", signed=False, parent=self)
        self.HS_SHORT_EN_Y2         = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "MCC.GDRV_STATUS_EN.HS_SHORT_EN_Y2", signed=False, parent=self)
        self.HS_VGS_OFF_SHORT_EN_U  = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "MCC.GDRV_STATUS_EN.HS_VGS_OFF_SHORT_EN_U", signed=False, parent=self)
        self.HS_VGS_OFF_SHORT_EN_V  = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "MCC.GDRV_STATUS_EN.HS_VGS_OFF_SHORT_EN_V", signed=False, parent=self)
        self.HS_VGS_OFF_SHORT_EN_W  = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "MCC.GDRV_STATUS_EN.HS_VGS_OFF_SHORT_EN_W", signed=False, parent=self)
        self.HS_VGS_OFF_SHORT_EN_Y2 = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "MCC.GDRV_STATUS_EN.HS_VGS_OFF_SHORT_EN_Y2", signed=False, parent=self)
        self.HS_VGS_ON_SHORT_EN_U   = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "MCC.GDRV_STATUS_EN.HS_VGS_ON_SHORT_EN_U", signed=False, parent=self)
        self.HS_VGS_ON_SHORT_EN_V   = Field(self.ADDRESS, self.WIDTH, 0x02000000, 25, "MCC.GDRV_STATUS_EN.HS_VGS_ON_SHORT_EN_V", signed=False, parent=self)
        self.HS_VGS_ON_SHORT_EN_W   = Field(self.ADDRESS, self.WIDTH, 0x04000000, 26, "MCC.GDRV_STATUS_EN.HS_VGS_ON_SHORT_EN_W", signed=False, parent=self)
        self.HS_VGS_ON_SHORT_EN_Y2  = Field(self.ADDRESS, self.WIDTH, 0x08000000, 27, "MCC.GDRV_STATUS_EN.HS_VGS_ON_SHORT_EN_Y2", signed=False, parent=self)
        self.VDRV_UVLO_EN           = Field(self.ADDRESS, self.WIDTH, 0x20000000, 29, "MCC.GDRV_STATUS_EN.VDRV_UVLO_EN", signed=False, parent=self)
        self.VDRV_UVLWRN_EN         = Field(self.ADDRESS, self.WIDTH, 0x40000000, 30, "MCC.GDRV_STATUS_EN.VDRV_UVLWRN_EN", signed=False, parent=self)
        self.VS_UVLO_EN             = Field(self.ADDRESS, self.WIDTH, 0x80000000, 31, "MCC.GDRV_STATUS_EN.VS_UVLO_EN", signed=False, parent=self)

class _Mcc_GdrvStatus(Register):
    ADDRESS = 0x4800A7C0
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.LS_SHORT_U          = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "MCC.GDRV_STATUS.LS_SHORT_U", signed=False, parent=self)
        self.LS_SHORT_V          = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "MCC.GDRV_STATUS.LS_SHORT_V", signed=False, parent=self)
        self.LS_SHORT_W          = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "MCC.GDRV_STATUS.LS_SHORT_W", signed=False, parent=self)
        self.LS_SHORT_Y2         = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "MCC.GDRV_STATUS.LS_SHORT_Y2", signed=False, parent=self)
        self.LS_VGS_OFF_SHORT_U  = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "MCC.GDRV_STATUS.LS_VGS_OFF_SHORT_U", signed=False, parent=self)
        self.LS_VGS_OFF_SHORT_V  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "MCC.GDRV_STATUS.LS_VGS_OFF_SHORT_V", signed=False, parent=self)
        self.LS_VGS_OFF_SHORT_W  = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "MCC.GDRV_STATUS.LS_VGS_OFF_SHORT_W", signed=False, parent=self)
        self.LS_VGS_OFF_SHORT_Y2 = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "MCC.GDRV_STATUS.LS_VGS_OFF_SHORT_Y2", signed=False, parent=self)
        self.LS_VGS_ON_SHORT_U   = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "MCC.GDRV_STATUS.LS_VGS_ON_SHORT_U", signed=False, parent=self)
        self.LS_VGS_ON_SHORT_V   = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "MCC.GDRV_STATUS.LS_VGS_ON_SHORT_V", signed=False, parent=self)
        self.LS_VGS_ON_SHORT_W   = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "MCC.GDRV_STATUS.LS_VGS_ON_SHORT_W", signed=False, parent=self)
        self.LS_VGS_ON_SHORT_Y2  = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "MCC.GDRV_STATUS.LS_VGS_ON_SHORT_Y2", signed=False, parent=self)
        self.BST_UVLO_U          = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "MCC.GDRV_STATUS.BST_UVLO_U", signed=False, parent=self)
        self.BST_UVLO_V          = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "MCC.GDRV_STATUS.BST_UVLO_V", signed=False, parent=self)
        self.BST_UVLO_W          = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "MCC.GDRV_STATUS.BST_UVLO_W", signed=False, parent=self)
        self.BST_UVLO_Y2         = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "MCC.GDRV_STATUS.BST_UVLO_Y2", signed=False, parent=self)
        self.HS_SHORT_U          = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "MCC.GDRV_STATUS.HS_SHORT_U", signed=False, parent=self)
        self.HS_SHORT_V          = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "MCC.GDRV_STATUS.HS_SHORT_V", signed=False, parent=self)
        self.HS_SHORT_W          = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "MCC.GDRV_STATUS.HS_SHORT_W", signed=False, parent=self)
        self.HS_SHORT_Y2         = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "MCC.GDRV_STATUS.HS_SHORT_Y2", signed=False, parent=self)
        self.HS_VGS_OFF_SHORT_U  = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "MCC.GDRV_STATUS.HS_VGS_OFF_SHORT_U", signed=False, parent=self)
        self.HS_VGS_OFF_SHORT_V  = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "MCC.GDRV_STATUS.HS_VGS_OFF_SHORT_V", signed=False, parent=self)
        self.HS_VGS_OFF_SHORT_W  = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "MCC.GDRV_STATUS.HS_VGS_OFF_SHORT_W", signed=False, parent=self)
        self.HS_VGS_OFF_SHORT_Y2 = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "MCC.GDRV_STATUS.HS_VGS_OFF_SHORT_Y2", signed=False, parent=self)
        self.HS_VGS_ON_SHORT_U   = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "MCC.GDRV_STATUS.HS_VGS_ON_SHORT_U", signed=False, parent=self)
        self.HS_VGS_ON_SHORT_V   = Field(self.ADDRESS, self.WIDTH, 0x02000000, 25, "MCC.GDRV_STATUS.HS_VGS_ON_SHORT_V", signed=False, parent=self)
        self.HS_VGS_ON_SHORT_W   = Field(self.ADDRESS, self.WIDTH, 0x04000000, 26, "MCC.GDRV_STATUS.HS_VGS_ON_SHORT_W", signed=False, parent=self)
        self.HS_VGS_ON_SHORT_Y2  = Field(self.ADDRESS, self.WIDTH, 0x08000000, 27, "MCC.GDRV_STATUS.HS_VGS_ON_SHORT_Y2", signed=False, parent=self)
        self.VDRV_UVLO           = Field(self.ADDRESS, self.WIDTH, 0x20000000, 29, "MCC.GDRV_STATUS.VDRV_UVLO", signed=False, parent=self)
        self.VDRV_UVLWRN         = Field(self.ADDRESS, self.WIDTH, 0x40000000, 30, "MCC.GDRV_STATUS.VDRV_UVLWRN", signed=False, parent=self)
        self.VS_UVLO             = Field(self.ADDRESS, self.WIDTH, 0x80000000, 31, "MCC.GDRV_STATUS.VS_UVLO", signed=False, parent=self)

class _Mcc_GdrvFault(Register):
    ADDRESS = 0x4800A7C4
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.LS_FAULT_ACTIVE_U  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "MCC.GDRV_FAULT.LS_FAULT_ACTIVE_U", signed=False, parent=self)
        self.LS_FAULT_ACTIVE_V  = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "MCC.GDRV_FAULT.LS_FAULT_ACTIVE_V", signed=False, parent=self)
        self.LS_FAULT_ACTIVE_W  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "MCC.GDRV_FAULT.LS_FAULT_ACTIVE_W", signed=False, parent=self)
        self.LS_FAULT_ACTIVE_Y2 = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "MCC.GDRV_FAULT.LS_FAULT_ACTIVE_Y2", signed=False, parent=self)
        self.BST_UVLO_STS_U     = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "MCC.GDRV_FAULT.BST_UVLO_STS_U", signed=False, parent=self)
        self.BST_UVLO_STS_V     = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "MCC.GDRV_FAULT.BST_UVLO_STS_V", signed=False, parent=self)
        self.BST_UVLO_STS_W     = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "MCC.GDRV_FAULT.BST_UVLO_STS_W", signed=False, parent=self)
        self.BST_UVLO_STS_Y2    = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "MCC.GDRV_FAULT.BST_UVLO_STS_Y2", signed=False, parent=self)
        self.HS_FAULT_ACTIVE_U  = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "MCC.GDRV_FAULT.HS_FAULT_ACTIVE_U", signed=False, parent=self)
        self.HS_FAULT_ACTIVE_V  = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "MCC.GDRV_FAULT.HS_FAULT_ACTIVE_V", signed=False, parent=self)
        self.HS_FAULT_ACTIVE_W  = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "MCC.GDRV_FAULT.HS_FAULT_ACTIVE_W", signed=False, parent=self)
        self.HS_FAULT_ACTIVE_Y2 = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "MCC.GDRV_FAULT.HS_FAULT_ACTIVE_Y2", signed=False, parent=self)
        self.VDRV_UVLO_STS      = Field(self.ADDRESS, self.WIDTH, 0x20000000, 29, "MCC.GDRV_FAULT.VDRV_UVLO_STS", signed=False, parent=self)
        self.VDRV_UVLWRN_STS    = Field(self.ADDRESS, self.WIDTH, 0x40000000, 30, "MCC.GDRV_FAULT.VDRV_UVLWRN_STS", signed=False, parent=self)
        self.VS_UVLO_STS        = Field(self.ADDRESS, self.WIDTH, 0x80000000, 31, "MCC.GDRV_FAULT.VS_UVLO_STS", signed=False, parent=self)

class _Mcc_AdcI1I0Ext(Register):
    ADDRESS = 0x4800A800
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.I0 = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ADC_I1_I0_EXT.I0", signed=True, parent=self)
        self.I1 = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.ADC_I1_I0_EXT.I1", signed=True, parent=self)

class _Mcc_AdcI2Ext(Register):
    ADDRESS = 0x4800A804
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.I2 = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.ADC_I2_EXT.I2", signed=True, parent=self)

class _Mcc_PwmVx2Ux1Ext(Register):
    ADDRESS = 0x4800A808
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.UX1 = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PWM_VX2_UX1_EXT.UX1", signed=False, parent=self)
        self.VX2 = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.PWM_VX2_UX1_EXT.VX2", signed=False, parent=self)

class _Mcc_PwmY2Wy1Ext(Register):
    ADDRESS = 0x4800A80C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.WY1 = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PWM_Y2_WY1_EXT.WY1", signed=False, parent=self)
        self.Y2  = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.PWM_Y2_WY1_EXT.Y2", signed=False, parent=self)

class _Mcc_PwmExtY2Alt(Register):
    ADDRESS = 0x4800A810
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PWM_EXT_Y2_ALT = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PWM_EXT_Y2_ALT.PWM_EXT_Y2_ALT", signed=False, parent=self)

class _Mcc_VoltageExt(Register):
    ADDRESS = 0x4800A814
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.UD = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.VOLTAGE_EXT.UD", signed=True, parent=self)
        self.UQ = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.VOLTAGE_EXT.UQ", signed=True, parent=self)

class _Mcc_PhiExt(Register):
    ADDRESS = 0x4800A818
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PHI_E_EXT = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "MCC.PHI_EXT.PHI_E_EXT", signed=True, parent=self)
        self.PHI_M_EXT = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "MCC.PHI_EXT.PHI_M_EXT", signed=True, parent=self)

class _Mcc_VelocityExt(Register):
    ADDRESS = 0x4800A820
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.VELOCITY_EXT = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "MCC.VELOCITY_EXT.VELOCITY_EXT", signed=True, parent=self)


class _I2c(AddressBlock):
    _NAME       = "I2C"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.CPR    = _I2c_Cpr(self)
        self.CTRL   = _I2c_Ctrl(self)
        self.RX     = _I2c_Rx(self)
        self.STATUS = _I2c_Status(self)
        self.TX     = _I2c_Tx(self)
        self.CMD    = _I2c_Cmd(self)


class _I2c_Cpr(Register):
    ADDRESS = 0x4800B000
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PRE = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "I2C.CPR.PRE", signed=False, parent=self)

class _I2c_Ctrl(Register):
    ADDRESS = 0x4800B004
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.IE = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "I2C.CTRL.IE", signed=False, parent=self)
        self.EN = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "I2C.CTRL.EN", signed=False, parent=self)

class _I2c_Rx(Register):
    ADDRESS = 0x4800B008
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RX = Field(self.ADDRESS, self.WIDTH, 0x000000FF,  0, "I2C.RX.RX", signed=False, parent=self)

class _I2c_Status(Register):
    ADDRESS = 0x4800B00C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.IRQ = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "I2C.STATUS.IRQ", signed=False, parent=self)
        self.TIP = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "I2C.STATUS.TIP", signed=False, parent=self)
        self.AL  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "I2C.STATUS.AL", signed=False, parent=self)
        self.BUS = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "I2C.STATUS.BUS", signed=False, parent=self)
        self.RXA = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "I2C.STATUS.RXA", signed=False, parent=self)

class _I2c_Tx(Register):
    ADDRESS = 0x4800B010
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.TX = Field(self.ADDRESS, self.WIDTH, 0x000000FF,  0, "I2C.TX.TX", signed=False, parent=self)

class _I2c_Cmd(Register):
    ADDRESS = 0x4800B014
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.IA  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "I2C.CMD.IA", signed=False, parent=self)
        self.ACK = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "I2C.CMD.ACK", signed=False, parent=self)
        self.WR  = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "I2C.CMD.WR", signed=False, parent=self)
        self.RD  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "I2C.CMD.RD", signed=False, parent=self)
        self.STO = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "I2C.CMD.STO", signed=False, parent=self)
        self.STA = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "I2C.CMD.STA", signed=False, parent=self)


class _Spi1(AddressBlock):
    _NAME       = "SPI1"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.STATUS           = _Spi1_Status(self)
        self.GEN_CONFIG       = _Spi1_GenConfig(self)
        self.SCLK_CONFIG0     = _Spi1_SclkConfig0(self)
        self.SYNC_CONFIG0     = _Spi1_SyncConfig0(self)
        self.TRANSFER_CONFIG0 = _Spi1_TransferConfig0(self)
        self.RD_BUFFER0       = _Spi1_RdBuffer0(self)
        self.RD_BUFFER1       = _Spi1_RdBuffer1(self)
        self.RD_BUFFER2       = _Spi1_RdBuffer2(self)
        self.RD_BUFFER3       = _Spi1_RdBuffer3(self)
        self.WR_BUFFER0       = _Spi1_WrBuffer0(self)
        self.WR_BUFFER1       = _Spi1_WrBuffer1(self)
        self.WR_BUFFER2       = _Spi1_WrBuffer2(self)
        self.WR_BUFFER3       = _Spi1_WrBuffer3(self)


class _Spi1_Status(Register):
    ADDRESS = 0x4800D000
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.TRANSACT_DONE                  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "SPI1.STATUS.TRANSACT_DONE", signed=False, parent=self)
        self.TRANSACT_STARTED               = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "SPI1.STATUS.TRANSACT_STARTED", signed=False, parent=self)
        self.TRANSACT_DONE_SINCE_STATUS_CLR = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "SPI1.STATUS.TRANSACT_DONE_SINCE_STATUS_CLR", signed=False, parent=self)
        self.BUSY1                          = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "SPI1.STATUS.BUSY1", signed=False, parent=self)
        self.BUSY2                          = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "SPI1.STATUS.BUSY2", signed=False, parent=self)
        self.CONT_ON                        = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "SPI1.STATUS.CONT_ON", signed=False, parent=self)
        self.RX_CORK                        = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "SPI1.STATUS.RX_CORK", signed=False, parent=self)

class _Spi1_GenConfig(Register):
    ADDRESS = 0x4800D004
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.ENABLE           = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "SPI1.GEN_CONFIG.ENABLE", signed=False, parent=self)
        self.CONFIG_SEL       = Field(self.ADDRESS, self.WIDTH, 0x00000006,  1, "SPI1.GEN_CONFIG.CONFIG_SEL", signed=False, parent=self)
        self.START_NSTOP      = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "SPI1.GEN_CONFIG.START_NSTOP", signed=False, parent=self)
        self.START_PULSE      = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "SPI1.GEN_CONFIG.START_PULSE", signed=False, parent=self)
        self.READ_REQUEST     = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "SPI1.GEN_CONFIG.READ_REQUEST", signed=False, parent=self)
        self.READ_REQUEST_CLR = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "SPI1.GEN_CONFIG.READ_REQUEST_CLR", signed=False, parent=self)

class _Spi1_SclkConfig0(Register):
    ADDRESS = 0x4800D008
    WIDTH = 4
    DEFAULT = 0x00000010

    def __init__(self, parent):
        self._PARENT = parent

        self.CLK_DIV       = Field(self.ADDRESS, self.WIDTH, 0x000000FF,  0, "SPI1.SCLK_CONFIG0.CLK_DIV", signed=False, parent=self)
        self.CS_SETTLE_TIM = Field(self.ADDRESS, self.WIDTH, 0x0000FF00,  8, "SPI1.SCLK_CONFIG0.CS_SETTLE_TIM", signed=False, parent=self)
        self.PAUSE_TIM     = Field(self.ADDRESS, self.WIDTH, 0x00FF0000, 16, "SPI1.SCLK_CONFIG0.PAUSE_TIM", signed=False, parent=self)

class _Spi1_SyncConfig0(Register):
    ADDRESS = 0x4800D00C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.STROBE_MASK      = Field(self.ADDRESS, self.WIDTH, 0x000001FF,  0, "SPI1.SYNC_CONFIG0.STROBE_MASK", signed=False, parent=self)
        self.SW_STROBE_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "SPI1.SYNC_CONFIG0.SW_STROBE_ENABLE", signed=False, parent=self)
        self.HW_STROBE_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "SPI1.SYNC_CONFIG0.HW_STROBE_ENABLE", signed=False, parent=self)
        self.SEC_CYCLE_ACTIVE = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "SPI1.SYNC_CONFIG0.SEC_CYCLE_ACTIVE", signed=False, parent=self)

class _Spi1_TransferConfig0(Register):
    ADDRESS = 0x4800D010
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.SCLK_PHASE      = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "SPI1.TRANSFER_CONFIG0.SCLK_PHASE", signed=False, parent=self)
        self.SCLK_POL        = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "SPI1.TRANSFER_CONFIG0.SCLK_POL", signed=False, parent=self)
        self.CS_POL          = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "SPI1.TRANSFER_CONFIG0.CS_POL", signed=False, parent=self)
        self.CS0_ENABLE      = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "SPI1.TRANSFER_CONFIG0.CS0_ENABLE", signed=False, parent=self)
        self.CS1_ENABLE      = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "SPI1.TRANSFER_CONFIG0.CS1_ENABLE", signed=False, parent=self)
        self.CS2_ENABLE      = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "SPI1.TRANSFER_CONFIG0.CS2_ENABLE", signed=False, parent=self)
        self.APPLY_TAD       = Field(self.ADDRESS, self.WIDTH, 0x000000C0,  6, "SPI1.TRANSFER_CONFIG0.APPLY_TAD", signed=False, parent=self)
        self.DATA_SIZE_SEL_0 = Field(self.ADDRESS, self.WIDTH, 0x00000F00,  8, "SPI1.TRANSFER_CONFIG0.DATA_SIZE_SEL_0", signed=False, parent=self)
        self.DATA_SIZE_SEL_1 = Field(self.ADDRESS, self.WIDTH, 0x0000F000, 12, "SPI1.TRANSFER_CONFIG0.DATA_SIZE_SEL_1", signed=False, parent=self)

class _Spi1_RdBuffer0(Register):
    ADDRESS = 0x4800D02C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RD_BUFFER0 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPI1.RD_BUFFER0.RD_BUFFER0", signed=False, parent=self)

class _Spi1_RdBuffer1(Register):
    ADDRESS = 0x4800D030
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RD_BUFFER1 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPI1.RD_BUFFER1.RD_BUFFER1", signed=False, parent=self)

class _Spi1_RdBuffer2(Register):
    ADDRESS = 0x4800D034
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RD_BUFFER2 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPI1.RD_BUFFER2.RD_BUFFER2", signed=False, parent=self)

class _Spi1_RdBuffer3(Register):
    ADDRESS = 0x4800D038
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RD_BUFFER3 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPI1.RD_BUFFER3.RD_BUFFER3", signed=False, parent=self)

class _Spi1_WrBuffer0(Register):
    ADDRESS = 0x4800D03C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.WR_BUFFER0 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPI1.WR_BUFFER0.WR_BUFFER0", signed=False, parent=self)

class _Spi1_WrBuffer1(Register):
    ADDRESS = 0x4800D040
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.WR_BUFFER1 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPI1.WR_BUFFER1.WR_BUFFER1", signed=False, parent=self)

class _Spi1_WrBuffer2(Register):
    ADDRESS = 0x4800D044
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.WR_BUFFER2 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPI1.WR_BUFFER2.WR_BUFFER2", signed=False, parent=self)

class _Spi1_WrBuffer3(Register):
    ADDRESS = 0x4800D048
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.WR_BUFFER3 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPI1.WR_BUFFER3.WR_BUFFER3", signed=False, parent=self)


class _SpiSlave(AddressBlock):
    _NAME       = "SPISLAVE"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.RX_BUFFER_L = _SpiSlave_RxBufferL(self)
        self.RX_BUFFER_H = _SpiSlave_RxBufferH(self)
        self.TX_BUFFER_L = _SpiSlave_TxBufferL(self)
        self.TX_BUFFER_H = _SpiSlave_TxBufferH(self)
        self.CFG         = _SpiSlave_Cfg(self)
        self.CTRL        = _SpiSlave_Ctrl(self)
        self.IRQ_EN      = _SpiSlave_IrqEn(self)
        self.STATUS      = _SpiSlave_Status(self)


class _SpiSlave_RxBufferL(Register):
    ADDRESS = 0x4800E000
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RX_BUFFER_L = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPISLAVE.RX_BUFFER_L.RX_BUFFER_L", signed=False, parent=self)

class _SpiSlave_RxBufferH(Register):
    ADDRESS = 0x4800E004
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RX_BUFFER_H = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPISLAVE.RX_BUFFER_H.RX_BUFFER_H", signed=False, parent=self)

class _SpiSlave_TxBufferL(Register):
    ADDRESS = 0x4800E008
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.TX_BUFFER_L = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPISLAVE.TX_BUFFER_L.TX_BUFFER_L", signed=False, parent=self)

class _SpiSlave_TxBufferH(Register):
    ADDRESS = 0x4800E00C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.TX_BUFFER_H = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "SPISLAVE.TX_BUFFER_H.TX_BUFFER_H", signed=False, parent=self)

class _SpiSlave_Cfg(Register):
    ADDRESS = 0x4800E010
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.SPI_SELECTION   = Field(self.ADDRESS, self.WIDTH, 0x00000003,  0, "SPISLAVE.CFG.SPI_SELECTION", signed=False, parent=self)
        self.SPI_DATA_LENGTH = Field(self.ADDRESS, self.WIDTH, 0x0000001C,  2, "SPISLAVE.CFG.SPI_DATA_LENGTH", signed=False, parent=self)
        self.RX_OVERRUN      = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "SPISLAVE.CFG.RX_OVERRUN", signed=False, parent=self)
        self.TX_OVERRUN      = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "SPISLAVE.CFG.TX_OVERRUN", signed=False, parent=self)
        self.DAISY_CHAIN_EN  = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "SPISLAVE.CFG.DAISY_CHAIN_EN", signed=False, parent=self)

class _SpiSlave_Ctrl(Register):
    ADDRESS = 0x4800E014
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.TX_BUFFER_RESET = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "SPISLAVE.CTRL.TX_BUFFER_RESET", signed=False, parent=self)

class _SpiSlave_IrqEn(Register):
    ADDRESS = 0x4800E018
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.EOT_IRQ_EN = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "SPISLAVE.IRQ_EN.EOT_IRQ_EN", signed=False, parent=self)
        self.SOT_IRQ_EN = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "SPISLAVE.IRQ_EN.SOT_IRQ_EN", signed=False, parent=self)

class _SpiSlave_Status(Register):
    ADDRESS = 0x4800E01C
    WIDTH = 4
    DEFAULT = 0x00000012

    def __init__(self, parent):
        self._PARENT = parent

        self.EOT                  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "SPISLAVE.STATUS.EOT", signed=False, parent=self)
        self.TX_BUFFER_NOT_FULL   = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "SPISLAVE.STATUS.TX_BUFFER_NOT_FULL", signed=False, parent=self)
        self.RX_BUFFER_NOT_EMPTY  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "SPISLAVE.STATUS.RX_BUFFER_NOT_EMPTY", signed=False, parent=self)
        self.RX_DATA_LOST         = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "SPISLAVE.STATUS.RX_DATA_LOST", signed=False, parent=self)
        self.TX_BUFFER_EMPTY      = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "SPISLAVE.STATUS.TX_BUFFER_EMPTY", signed=False, parent=self)
        self.BUSY                 = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "SPISLAVE.STATUS.BUSY", signed=False, parent=self)
        self.START_OF_TRANSACTION = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "SPISLAVE.STATUS.START_OF_TRANSACTION", signed=False, parent=self)


class _Cordic(AddressBlock):
    _NAME       = "CORDIC"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.CTRL       = _Cordic_Ctrl(self)
        self.STATUS     = _Cordic_Status(self)
        self.X_DATA_IN  = _Cordic_XDataIn(self)
        self.Y_DATA_IN  = _Cordic_YDataIn(self)
        self.Z_DATA_IN  = _Cordic_ZDataIn(self)
        self.X_DATA_OUT = _Cordic_XDataOut(self)
        self.Y_DATA_OUT = _Cordic_YDataOut(self)
        self.Z_DATA_OUT = _Cordic_ZDataOut(self)


class _Cordic_Ctrl(Register):
    ADDRESS = 0x4800F000
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.START_ROT       = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "CORDIC.CTRL.START_ROT", signed=False, parent=self)
        self.START_VECT      = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "CORDIC.CTRL.START_VECT", signed=False, parent=self)
        self.START_VECT_ATAN = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "CORDIC.CTRL.START_VECT_ATAN", signed=False, parent=self)

class _Cordic_Status(Register):
    ADDRESS = 0x4800F004
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.OP_DONE    = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "CORDIC.STATUS.OP_DONE", signed=False, parent=self)
        self.OP_CLEAR   = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "CORDIC.STATUS.OP_CLEAR", signed=False, parent=self)
        self.CURRENT_OP = Field(self.ADDRESS, self.WIDTH, 0x0000000C,  2, "CORDIC.STATUS.CURRENT_OP", signed=False, parent=self)

class _Cordic_XDataIn(Register):
    ADDRESS = 0x4800F008
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.X_DATA_IN = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "CORDIC.X_DATA_IN.X_DATA_IN", signed=False, parent=self)

class _Cordic_YDataIn(Register):
    ADDRESS = 0x4800F00C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.Y_DATA_IN = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "CORDIC.Y_DATA_IN.Y_DATA_IN", signed=False, parent=self)

class _Cordic_ZDataIn(Register):
    ADDRESS = 0x4800F010
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.Z_DATA_IN = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "CORDIC.Z_DATA_IN.Z_DATA_IN", signed=False, parent=self)

class _Cordic_XDataOut(Register):
    ADDRESS = 0x4800F014
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.X_DATA_OUT = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "CORDIC.X_DATA_OUT.X_DATA_OUT", signed=False, parent=self)

class _Cordic_YDataOut(Register):
    ADDRESS = 0x4800F018
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.Y_DATA_OUT = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "CORDIC.Y_DATA_OUT.Y_DATA_OUT", signed=False, parent=self)

class _Cordic_ZDataOut(Register):
    ADDRESS = 0x4800F01C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.Z_DATA_OUT = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "CORDIC.Z_DATA_OUT.Z_DATA_OUT", signed=False, parent=self)


class _Uart(AddressBlock):
    _NAME       = "UART"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.TX_BUFFER = _Uart_TxBuffer(self)
        self.BAUDRATE  = _Uart_Baudrate(self)
        self.CONTROL   = _Uart_Control(self)
        self.PATTERN   = _Uart_Pattern(self)
        self.TIMEOUT   = _Uart_Timeout(self)
        self.IRQ_EN    = _Uart_IrqEn(self)
        self.STATUS    = _Uart_Status(self)


class _Uart_TxBuffer(Register):
    ADDRESS = 0x48010000
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.DATA       = Field(self.ADDRESS, self.WIDTH, 0x000000FF,  0, "UART.TX_BUFFER.DATA", signed=False, parent=self)
        self.ADDR_BREAK = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "UART.TX_BUFFER.ADDR_BREAK", signed=False, parent=self)

class _Uart_Baudrate(Register):
    ADDRESS = 0x48010004
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.MANTISSA_LIMIT = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "UART.BAUDRATE.MANTISSA_LIMIT", signed=False, parent=self)
        self.FRACTION_LIMIT = Field(self.ADDRESS, self.WIDTH, 0x000F0000, 16, "UART.BAUDRATE.FRACTION_LIMIT", signed=False, parent=self)
        self.ENABLE         = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "UART.BAUDRATE.ENABLE", signed=False, parent=self)

class _Uart_Control(Register):
    ADDRESS = 0x48010008
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.DATA_LENGTH                = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "UART.CONTROL.DATA_LENGTH", signed=False, parent=self)
        self.STOP_BITS                  = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "UART.CONTROL.STOP_BITS", signed=False, parent=self)
        self.ADDR_ENABLE                = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "UART.CONTROL.ADDR_ENABLE", signed=False, parent=self)
        self.ADDR_MUTE                  = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "UART.CONTROL.ADDR_MUTE", signed=False, parent=self)
        self.PARITY_ENABLE              = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "UART.CONTROL.PARITY_ENABLE", signed=False, parent=self)
        self.PARITY_EVEN                = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "UART.CONTROL.PARITY_EVEN", signed=False, parent=self)
        self.OVER8                      = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "UART.CONTROL.OVER8", signed=False, parent=self)
        self.BREAK_ENABLE               = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "UART.CONTROL.BREAK_ENABLE", signed=False, parent=self)
        self.FILTER_ENABLE              = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "UART.CONTROL.FILTER_ENABLE", signed=False, parent=self)
        self.AUTOBAUD_ENABLE            = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "UART.CONTROL.AUTOBAUD_ENABLE", signed=False, parent=self)
        self.AUTOBAUD_MODE              = Field(self.ADDRESS, self.WIDTH, 0x00000C00, 10, "UART.CONTROL.AUTOBAUD_MODE", signed=False, parent=self)
        self.BREAK_AUTO                 = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "UART.CONTROL.BREAK_AUTO", signed=False, parent=self)
        self.LOOPBACK                   = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "UART.CONTROL.LOOPBACK", signed=False, parent=self)
        self.TIMEOUT_PRE_DIVIDER_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "UART.CONTROL.TIMEOUT_PRE_DIVIDER_ENABLE", signed=False, parent=self)
        self.PRE_TXD_EN_LIMIT           = Field(self.ADDRESS, self.WIDTH, 0x00FF0000, 16, "UART.CONTROL.PRE_TXD_EN_LIMIT", signed=False, parent=self)
        self.POST_TXD_EN_LIMIT          = Field(self.ADDRESS, self.WIDTH, 0xFF000000, 24, "UART.CONTROL.POST_TXD_EN_LIMIT", signed=False, parent=self)

class _Uart_Pattern(Register):
    ADDRESS = 0x4801000C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.PATTERN0              = Field(self.ADDRESS, self.WIDTH, 0x000000FF,  0, "UART.PATTERN.PATTERN0", signed=False, parent=self)
        self.PATTERN1              = Field(self.ADDRESS, self.WIDTH, 0x0000FF00,  8, "UART.PATTERN.PATTERN1", signed=False, parent=self)
        self.RX_BUFFER_RESET       = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "UART.PATTERN.RX_BUFFER_RESET", signed=False, parent=self)
        self.TX_BUFFER_RESET       = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "UART.PATTERN.TX_BUFFER_RESET", signed=False, parent=self)
        self.RX_MESSAGE_SIZE       = Field(self.ADDRESS, self.WIDTH, 0x003C0000, 18, "UART.PATTERN.RX_MESSAGE_SIZE", signed=False, parent=self)
        self.RX_MESSAGE_ADDR_INDEX = Field(self.ADDRESS, self.WIDTH, 0x03C00000, 22, "UART.PATTERN.RX_MESSAGE_ADDR_INDEX", signed=False, parent=self)

class _Uart_Timeout(Register):
    ADDRESS = 0x48010010
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.LIMIT   = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "UART.TIMEOUT.LIMIT", signed=False, parent=self)
        self.COUNTER = Field(self.ADDRESS, self.WIDTH, 0xFFFF0000, 16, "UART.TIMEOUT.COUNTER", signed=False, parent=self)

class _Uart_IrqEn(Register):
    ADDRESS = 0x48010014
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RX_NEW_DATA             = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "UART.IRQ_EN.RX_NEW_DATA", signed=False, parent=self)
        self.RX_MESSAGE_COMPLETE_INT = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "UART.IRQ_EN.RX_MESSAGE_COMPLETE_INT", signed=False, parent=self)
        self.RX_BUFFER_OVERFLOW_INT  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "UART.IRQ_EN.RX_BUFFER_OVERFLOW_INT", signed=False, parent=self)
        self.TX_BUFFER_EMPTY_INT     = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "UART.IRQ_EN.TX_BUFFER_EMPTY_INT", signed=False, parent=self)
        self.TX_ACTIVE_END_INT       = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "UART.IRQ_EN.TX_ACTIVE_END_INT", signed=False, parent=self)
        self.NOISY_DATA_INT          = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "UART.IRQ_EN.NOISY_DATA_INT", signed=False, parent=self)
        self.START_BIT_ERROR_INT     = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "UART.IRQ_EN.START_BIT_ERROR_INT", signed=False, parent=self)
        self.PARITY_ERROR_INT        = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "UART.IRQ_EN.PARITY_ERROR_INT", signed=False, parent=self)
        self.FRAMING_ERROR_INT       = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "UART.IRQ_EN.FRAMING_ERROR_INT", signed=False, parent=self)
        self.ADDR_INT                = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "UART.IRQ_EN.ADDR_INT", signed=False, parent=self)
        self.PATTERN0_MATCH_INT      = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "UART.IRQ_EN.PATTERN0_MATCH_INT", signed=False, parent=self)
        self.PATTERN1_MATCH_INT      = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "UART.IRQ_EN.PATTERN1_MATCH_INT", signed=False, parent=self)
        self.PATTERN01_MATCH_INT     = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "UART.IRQ_EN.PATTERN01_MATCH_INT", signed=False, parent=self)
        self.BREAK_INT               = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "UART.IRQ_EN.BREAK_INT", signed=False, parent=self)
        self.AUTOBAUD_COMPLETE_INT   = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "UART.IRQ_EN.AUTOBAUD_COMPLETE_INT", signed=False, parent=self)
        self.AUTOBAUD_ERROR_INT      = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "UART.IRQ_EN.AUTOBAUD_ERROR_INT", signed=False, parent=self)
        self.RX_IDLE_TIMEOUT_INT     = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "UART.IRQ_EN.RX_IDLE_TIMEOUT_INT", signed=False, parent=self)

class _Uart_Status(Register):
    ADDRESS = 0x48010018
    WIDTH = 4
    DEFAULT = 0x04000008

    def __init__(self, parent):
        self._PARENT = parent

        self.RX_NEW_DATA_FLAG         = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "UART.STATUS.RX_NEW_DATA_FLAG", signed=False, parent=self)
        self.RX_MESSAGE_COMPLETE_FLAG = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "UART.STATUS.RX_MESSAGE_COMPLETE_FLAG", signed=False, parent=self)
        self.RX_BUFFER_OVERFLOW_FLAG  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "UART.STATUS.RX_BUFFER_OVERFLOW_FLAG", signed=False, parent=self)
        self.TX_BUFFER_EMPTY_FLAG     = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "UART.STATUS.TX_BUFFER_EMPTY_FLAG", signed=False, parent=self)
        self.TX_ACTIVE_END_FLAG       = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "UART.STATUS.TX_ACTIVE_END_FLAG", signed=False, parent=self)
        self.NOISY_DATA_FLAG          = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "UART.STATUS.NOISY_DATA_FLAG", signed=False, parent=self)
        self.START_BIT_ERROR_FLAG     = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "UART.STATUS.START_BIT_ERROR_FLAG", signed=False, parent=self)
        self.PARITY_ERROR_FLAG        = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "UART.STATUS.PARITY_ERROR_FLAG", signed=False, parent=self)
        self.FRAMING_ERROR_FLAG       = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "UART.STATUS.FRAMING_ERROR_FLAG", signed=False, parent=self)
        self.ADDR_FLAG                = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "UART.STATUS.ADDR_FLAG", signed=False, parent=self)
        self.PATTERN0_MATCH_FLAG      = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "UART.STATUS.PATTERN0_MATCH_FLAG", signed=False, parent=self)
        self.PATTERN1_MATCH_FLAG      = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "UART.STATUS.PATTERN1_MATCH_FLAG", signed=False, parent=self)
        self.PATTERN01_MATCH_FLAG     = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "UART.STATUS.PATTERN01_MATCH_FLAG", signed=False, parent=self)
        self.BREAK_FLAG               = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "UART.STATUS.BREAK_FLAG", signed=False, parent=self)
        self.AUTOBAUD_COMPLETE_FLAG   = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "UART.STATUS.AUTOBAUD_COMPLETE_FLAG", signed=False, parent=self)
        self.AUTOBAUD_ERROR_FLAG      = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "UART.STATUS.AUTOBAUD_ERROR_FLAG", signed=False, parent=self)
        self.RX_IDLE_TIMEOUT_FLAG     = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "UART.STATUS.RX_IDLE_TIMEOUT_FLAG", signed=False, parent=self)
        self.RX_ACTIVE                = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "UART.STATUS.RX_ACTIVE", signed=False, parent=self)
        self.TX_ACTIVE                = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "UART.STATUS.TX_ACTIVE", signed=False, parent=self)
        self.RX_BUFFER_ENTRIES        = Field(self.ADDRESS, self.WIDTH, 0x00780000, 19, "UART.STATUS.RX_BUFFER_ENTRIES", signed=False, parent=self)
        self.TX_BUFFER_AVAILABLE      = Field(self.ADDRESS, self.WIDTH, 0x07800000, 23, "UART.STATUS.TX_BUFFER_AVAILABLE", signed=False, parent=self)


class _TimBasic0(AddressBlock):
    _NAME       = "TIMBASIC0"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.COUNTER  = _TimBasic0_Counter(self)
        self.LIMIT    = _TimBasic0_Limit(self)
        self.COMPARE  = _TimBasic0_Compare(self)
        self.CONFIG   = _TimBasic0_Config(self)
        self.IRQ_CTRL = _TimBasic0_IrqCtrl(self)
        self.STATUS   = _TimBasic0_Status(self)


class _TimBasic0_Counter(Register):
    ADDRESS = 0x48011000
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COUNTER = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMBASIC0.COUNTER.COUNTER", signed=False, parent=self)

class _TimBasic0_Limit(Register):
    ADDRESS = 0x48011004
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.LIMIT = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMBASIC0.LIMIT.LIMIT", signed=False, parent=self)

class _TimBasic0_Compare(Register):
    ADDRESS = 0x48011008
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COMPARE = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMBASIC0.COMPARE.COMPARE", signed=False, parent=self)

class _TimBasic0_Config(Register):
    ADDRESS = 0x4801100C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.ENABLE     = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "TIMBASIC0.CONFIG.ENABLE", signed=False, parent=self)
        self.PWM_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "TIMBASIC0.CONFIG.PWM_ENABLE", signed=False, parent=self)
        self.TOGGLE     = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "TIMBASIC0.CONFIG.TOGGLE", signed=False, parent=self)

class _TimBasic0_IrqCtrl(Register):
    ADDRESS = 0x48011010
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.OVER_IRQ_EN      = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "TIMBASIC0.IRQ_CTRL.OVER_IRQ_EN", signed=False, parent=self)
        self.CMP_MATCH_IRQ_EN = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "TIMBASIC0.IRQ_CTRL.CMP_MATCH_IRQ_EN", signed=False, parent=self)
        self.STOP_IRQ_EN      = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "TIMBASIC0.IRQ_CTRL.STOP_IRQ_EN", signed=False, parent=self)

class _TimBasic0_Status(Register):
    ADDRESS = 0x48011014
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.OVER_FLAG  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "TIMBASIC0.STATUS.OVER_FLAG", signed=False, parent=self)
        self.MATCH_FLAG = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "TIMBASIC0.STATUS.MATCH_FLAG", signed=False, parent=self)
        self.STOP_FLAG  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "TIMBASIC0.STATUS.STOP_FLAG", signed=False, parent=self)


class _TimAdv(AddressBlock):
    _NAME       = "TIMADV"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.COUNTER0       = _TimAdv_Counter0(self)
        self.LIMIT0         = _TimAdv_Limit0(self)
        self.CONFIG0        = _TimAdv_Config0(self)
        self.COUNTER1       = _TimAdv_Counter1(self)
        self.LIMIT1         = _TimAdv_Limit1(self)
        self.CONFIG1        = _TimAdv_Config1(self)
        self.COUNTER2       = _TimAdv_Counter2(self)
        self.LIMIT2         = _TimAdv_Limit2(self)
        self.CONFIG2        = _TimAdv_Config2(self)
        self.CAPTURE0       = _TimAdv_Capture0(self)
        self.CAPTURE1       = _TimAdv_Capture1(self)
        self.CAPTURE2       = _TimAdv_Capture2(self)
        self.CAPTURE_CONFIG = _TimAdv_CaptureConfig(self)
        self.COMPARE0       = _TimAdv_Compare0(self)
        self.COMPARE1       = _TimAdv_Compare1(self)
        self.OUT_CONFIG     = _TimAdv_OutConfig(self)
        self.IN_CONFIG      = _TimAdv_InConfig(self)
        self.IRQ_CTRL       = _TimAdv_IrqCtrl(self)
        self.STATUS         = _TimAdv_Status(self)


class _TimAdv_Counter0(Register):
    ADDRESS = 0x48012000
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COUNTER0_VALUE = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMADV.COUNTER0.COUNTER0_VALUE", signed=False, parent=self)

class _TimAdv_Limit0(Register):
    ADDRESS = 0x48012004
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COUNTER0_LIMIT = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMADV.LIMIT0.COUNTER0_LIMIT", signed=False, parent=self)

class _TimAdv_Config0(Register):
    ADDRESS = 0x48012008
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.ENABLE          = Field(self.ADDRESS, self.WIDTH, 0x00000003,  0, "TIMADV.CONFIG0.ENABLE", signed=False, parent=self)
        self.DIR             = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "TIMADV.CONFIG0.DIR", signed=False, parent=self)
        self.UP_CLK_CFG0     = Field(self.ADDRESS, self.WIDTH, 0x000000F0,  4, "TIMADV.CONFIG0.UP_CLK_CFG0", signed=False, parent=self)
        self.UP_CLK_CFG1     = Field(self.ADDRESS, self.WIDTH, 0x00000700,  8, "TIMADV.CONFIG0.UP_CLK_CFG1", signed=False, parent=self)
        self.UP_CLK_ENABLE   = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "TIMADV.CONFIG0.UP_CLK_ENABLE", signed=False, parent=self)
        self.DOWN_CLK_CFG0   = Field(self.ADDRESS, self.WIDTH, 0x0000F000, 12, "TIMADV.CONFIG0.DOWN_CLK_CFG0", signed=False, parent=self)
        self.DOWN_CLK_CFG1   = Field(self.ADDRESS, self.WIDTH, 0x00070000, 16, "TIMADV.CONFIG0.DOWN_CLK_CFG1", signed=False, parent=self)
        self.DOWN_CLK_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "TIMADV.CONFIG0.DOWN_CLK_ENABLE", signed=False, parent=self)
        self.RESET_ENABLE    = Field(self.ADDRESS, self.WIDTH, 0x00300000, 20, "TIMADV.CONFIG0.RESET_ENABLE", signed=False, parent=self)
        self.RESET_CFG_IN0   = Field(self.ADDRESS, self.WIDTH, 0x03800000, 23, "TIMADV.CONFIG0.RESET_CFG_IN0", signed=False, parent=self)
        self.RESET_CFG_IN1   = Field(self.ADDRESS, self.WIDTH, 0x1C000000, 26, "TIMADV.CONFIG0.RESET_CFG_IN1", signed=False, parent=self)
        self.RESET_CFG_IN2   = Field(self.ADDRESS, self.WIDTH, 0xE0000000, 29, "TIMADV.CONFIG0.RESET_CFG_IN2", signed=False, parent=self)

class _TimAdv_Counter1(Register):
    ADDRESS = 0x4801200C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COUNTER1_VALUE = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMADV.COUNTER1.COUNTER1_VALUE", signed=False, parent=self)

class _TimAdv_Limit1(Register):
    ADDRESS = 0x48012010
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COUNTER1_LIMIT = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMADV.LIMIT1.COUNTER1_LIMIT", signed=False, parent=self)

class _TimAdv_Config1(Register):
    ADDRESS = 0x48012014
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.ENABLE       = Field(self.ADDRESS, self.WIDTH, 0x00000007,  0, "TIMADV.CONFIG1.ENABLE", signed=False, parent=self)
        self.CLK_CFG      = Field(self.ADDRESS, self.WIDTH, 0x00000018,  3, "TIMADV.CONFIG1.CLK_CFG", signed=False, parent=self)
        self.RESET_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x00000060,  5, "TIMADV.CONFIG1.RESET_ENABLE", signed=False, parent=self)

class _TimAdv_Counter2(Register):
    ADDRESS = 0x48012018
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COUNTER2_VALUE = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMADV.COUNTER2.COUNTER2_VALUE", signed=False, parent=self)

class _TimAdv_Limit2(Register):
    ADDRESS = 0x4801201C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COUNTER2_LIMIT = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMADV.LIMIT2.COUNTER2_LIMIT", signed=False, parent=self)

class _TimAdv_Config2(Register):
    ADDRESS = 0x48012020
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.ENABLE  = Field(self.ADDRESS, self.WIDTH, 0x00000007,  0, "TIMADV.CONFIG2.ENABLE", signed=False, parent=self)
        self.CLK_CFG = Field(self.ADDRESS, self.WIDTH, 0x00000018,  3, "TIMADV.CONFIG2.CLK_CFG", signed=False, parent=self)

class _TimAdv_Capture0(Register):
    ADDRESS = 0x48012024
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.CAPTURE0 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMADV.CAPTURE0.CAPTURE0", signed=False, parent=self)

class _TimAdv_Capture1(Register):
    ADDRESS = 0x48012028
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.CAPTURE1 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMADV.CAPTURE1.CAPTURE1", signed=False, parent=self)

class _TimAdv_Capture2(Register):
    ADDRESS = 0x4801202C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.CAPTURE2 = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMADV.CAPTURE2.CAPTURE2", signed=False, parent=self)

class _TimAdv_CaptureConfig(Register):
    ADDRESS = 0x48012030
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.CAPTURE0_ENABLE         = Field(self.ADDRESS, self.WIDTH, 0x00000003,  0, "TIMADV.CAPTURE_CONFIG.CAPTURE0_ENABLE", signed=False, parent=self)
        self.CAPTURE0_IN0_CFG        = Field(self.ADDRESS, self.WIDTH, 0x0000000C,  2, "TIMADV.CAPTURE_CONFIG.CAPTURE0_IN0_CFG", signed=False, parent=self)
        self.CAPTURE0_IN1_LEVEL      = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "TIMADV.CAPTURE_CONFIG.CAPTURE0_IN1_LEVEL", signed=False, parent=self)
        self.CAPTURE0_IN1_EN         = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "TIMADV.CAPTURE_CONFIG.CAPTURE0_IN1_EN", signed=False, parent=self)
        self.CAPTURE0_IN2_LEVEL      = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "TIMADV.CAPTURE_CONFIG.CAPTURE0_IN2_LEVEL", signed=False, parent=self)
        self.CAPTURE0_IN2_EN         = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "TIMADV.CAPTURE_CONFIG.CAPTURE0_IN2_EN", signed=False, parent=self)
        self.CAPTURE1_ENABLE         = Field(self.ADDRESS, self.WIDTH, 0x00000300,  8, "TIMADV.CAPTURE_CONFIG.CAPTURE1_ENABLE", signed=False, parent=self)
        self.CAPTURE1_IN1_CFG        = Field(self.ADDRESS, self.WIDTH, 0x00000C00, 10, "TIMADV.CAPTURE_CONFIG.CAPTURE1_IN1_CFG", signed=False, parent=self)
        self.CAPTURE1_IN0_LEVEL      = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "TIMADV.CAPTURE_CONFIG.CAPTURE1_IN0_LEVEL", signed=False, parent=self)
        self.CAPTURE1_IN0_EN         = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "TIMADV.CAPTURE_CONFIG.CAPTURE1_IN0_EN", signed=False, parent=self)
        self.CAPTURE1_IN2_LEVEL      = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "TIMADV.CAPTURE_CONFIG.CAPTURE1_IN2_LEVEL", signed=False, parent=self)
        self.CAPTURE1_IN2_EN         = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "TIMADV.CAPTURE_CONFIG.CAPTURE1_IN2_EN", signed=False, parent=self)
        self.CAPTURE2_ENABLE         = Field(self.ADDRESS, self.WIDTH, 0x00030000, 16, "TIMADV.CAPTURE_CONFIG.CAPTURE2_ENABLE", signed=False, parent=self)
        self.CAPTURE2_IN2_CFG        = Field(self.ADDRESS, self.WIDTH, 0x000C0000, 18, "TIMADV.CAPTURE_CONFIG.CAPTURE2_IN2_CFG", signed=False, parent=self)
        self.CAPTURE2_IN0_LEVEL      = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "TIMADV.CAPTURE_CONFIG.CAPTURE2_IN0_LEVEL", signed=False, parent=self)
        self.CAPTURE2_IN0_EN         = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "TIMADV.CAPTURE_CONFIG.CAPTURE2_IN0_EN", signed=False, parent=self)
        self.CAPTURE2_IN1_LEVEL      = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "TIMADV.CAPTURE_CONFIG.CAPTURE2_IN1_LEVEL", signed=False, parent=self)
        self.CAPTURE2_IN1_EN         = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "TIMADV.CAPTURE_CONFIG.CAPTURE2_IN1_EN", signed=False, parent=self)
        self.CAPTURE1_COUNTER1_RESET = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "TIMADV.CAPTURE_CONFIG.CAPTURE1_COUNTER1_RESET", signed=False, parent=self)

class _TimAdv_Compare0(Register):
    ADDRESS = 0x48012034
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COMPARE0_VALUE = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMADV.COMPARE0.COMPARE0_VALUE", signed=False, parent=self)

class _TimAdv_Compare1(Register):
    ADDRESS = 0x48012038
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COMPARE1_VALUE = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMADV.COMPARE1.COMPARE1_VALUE", signed=False, parent=self)

class _TimAdv_OutConfig(Register):
    ADDRESS = 0x4801203C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.OUT0_CFG               = Field(self.ADDRESS, self.WIDTH, 0x00000007,  0, "TIMADV.OUT_CONFIG.OUT0_CFG", signed=False, parent=self)
        self.OUT0_COUNTER0_OVERFLOW = Field(self.ADDRESS, self.WIDTH, 0x00000018,  3, "TIMADV.OUT_CONFIG.OUT0_COUNTER0_OVERFLOW", signed=False, parent=self)
        self.OUT0_COMPARE0          = Field(self.ADDRESS, self.WIDTH, 0x00000060,  5, "TIMADV.OUT_CONFIG.OUT0_COMPARE0", signed=False, parent=self)
        self.OUT0_COMPARE1          = Field(self.ADDRESS, self.WIDTH, 0x00000180,  7, "TIMADV.OUT_CONFIG.OUT0_COMPARE1", signed=False, parent=self)
        self.OUT0_COUNTER1_OVERFLOW = Field(self.ADDRESS, self.WIDTH, 0x00000600,  9, "TIMADV.OUT_CONFIG.OUT0_COUNTER1_OVERFLOW", signed=False, parent=self)
        self.OUT1_CFG               = Field(self.ADDRESS, self.WIDTH, 0x00003800, 11, "TIMADV.OUT_CONFIG.OUT1_CFG", signed=False, parent=self)
        self.OUT1_COUNTER0_OVERFLOW = Field(self.ADDRESS, self.WIDTH, 0x0000C000, 14, "TIMADV.OUT_CONFIG.OUT1_COUNTER0_OVERFLOW", signed=False, parent=self)
        self.OUT1_COMPARE0          = Field(self.ADDRESS, self.WIDTH, 0x00030000, 16, "TIMADV.OUT_CONFIG.OUT1_COMPARE0", signed=False, parent=self)
        self.OUT1_COMPARE1          = Field(self.ADDRESS, self.WIDTH, 0x000C0000, 18, "TIMADV.OUT_CONFIG.OUT1_COMPARE1", signed=False, parent=self)
        self.OUT1_COUNTER2_OVERFLOW = Field(self.ADDRESS, self.WIDTH, 0x00300000, 20, "TIMADV.OUT_CONFIG.OUT1_COUNTER2_OVERFLOW", signed=False, parent=self)

class _TimAdv_InConfig(Register):
    ADDRESS = 0x48012040
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.IN0_EN              = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "TIMADV.IN_CONFIG.IN0_EN", signed=False, parent=self)
        self.IN0_FILTER_EN       = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "TIMADV.IN_CONFIG.IN0_FILTER_EN", signed=False, parent=self)
        self.IN1_EN              = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "TIMADV.IN_CONFIG.IN1_EN", signed=False, parent=self)
        self.IN1_FILTER_EN       = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "TIMADV.IN_CONFIG.IN1_FILTER_EN", signed=False, parent=self)
        self.IN1_USE_IN0_INSTEAD = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "TIMADV.IN_CONFIG.IN1_USE_IN0_INSTEAD", signed=False, parent=self)
        self.IN2_EN              = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "TIMADV.IN_CONFIG.IN2_EN", signed=False, parent=self)
        self.IN2_FILTER_EN       = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "TIMADV.IN_CONFIG.IN2_FILTER_EN", signed=False, parent=self)
        self.SAMPLE_RATE         = Field(self.ADDRESS, self.WIDTH, 0x00000380,  7, "TIMADV.IN_CONFIG.SAMPLE_RATE", signed=False, parent=self)

class _TimAdv_IrqCtrl(Register):
    ADDRESS = 0x48012044
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COUNTER0_OVERFLOW_INT_EN = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "TIMADV.IRQ_CTRL.COUNTER0_OVERFLOW_INT_EN", signed=False, parent=self)
        self.COUNTER1_OVERFLOW_INT_EN = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "TIMADV.IRQ_CTRL.COUNTER1_OVERFLOW_INT_EN", signed=False, parent=self)
        self.COUNTER2_OVERFLOW_INT_EN = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "TIMADV.IRQ_CTRL.COUNTER2_OVERFLOW_INT_EN", signed=False, parent=self)
        self.COUNTER0_STOPPED_INT_EN  = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "TIMADV.IRQ_CTRL.COUNTER0_STOPPED_INT_EN", signed=False, parent=self)
        self.COUNTER1_STOPPED_INT_EN  = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "TIMADV.IRQ_CTRL.COUNTER1_STOPPED_INT_EN", signed=False, parent=self)
        self.COUNTER2_STOPPED_INT_EN  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "TIMADV.IRQ_CTRL.COUNTER2_STOPPED_INT_EN", signed=False, parent=self)
        self.CAPTURE0_INT_EN          = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "TIMADV.IRQ_CTRL.CAPTURE0_INT_EN", signed=False, parent=self)
        self.CAPTURE1_INT_EN          = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "TIMADV.IRQ_CTRL.CAPTURE1_INT_EN", signed=False, parent=self)
        self.CAPTURE2_INT_EN          = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "TIMADV.IRQ_CTRL.CAPTURE2_INT_EN", signed=False, parent=self)
        self.COMPARE0_INT_EN          = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "TIMADV.IRQ_CTRL.COMPARE0_INT_EN", signed=False, parent=self)
        self.COMPARE1_INT_EN          = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "TIMADV.IRQ_CTRL.COMPARE1_INT_EN", signed=False, parent=self)

class _TimAdv_Status(Register):
    ADDRESS = 0x48012048
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COUNTER0_OVERFLOW_FLAG = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "TIMADV.STATUS.COUNTER0_OVERFLOW_FLAG", signed=False, parent=self)
        self.COUNTER1_OVERFLOW_FLAG = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "TIMADV.STATUS.COUNTER1_OVERFLOW_FLAG", signed=False, parent=self)
        self.COUNTER2_OVERFLOW_FLAG = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "TIMADV.STATUS.COUNTER2_OVERFLOW_FLAG", signed=False, parent=self)
        self.COUNTER0_STOPPED_FLAG  = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "TIMADV.STATUS.COUNTER0_STOPPED_FLAG", signed=False, parent=self)
        self.COUNTER1_STOPPED_FLAG  = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "TIMADV.STATUS.COUNTER1_STOPPED_FLAG", signed=False, parent=self)
        self.COUNTER2_STOPPED_FLAG  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "TIMADV.STATUS.COUNTER2_STOPPED_FLAG", signed=False, parent=self)
        self.CAPTURE0_FLAG          = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "TIMADV.STATUS.CAPTURE0_FLAG", signed=False, parent=self)
        self.CAPTURE1_FLAG          = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "TIMADV.STATUS.CAPTURE1_FLAG", signed=False, parent=self)
        self.CAPTURE2_FLAG          = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "TIMADV.STATUS.CAPTURE2_FLAG", signed=False, parent=self)
        self.COMPARE0_FLAG          = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "TIMADV.STATUS.COMPARE0_FLAG", signed=False, parent=self)
        self.COMPARE1_FLAG          = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "TIMADV.STATUS.COMPARE1_FLAG", signed=False, parent=self)


class _IoMatrix(AddressBlock):
    _NAME       = "IOMATRIX"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.PIN_7_0      = _IoMatrix_Pin70(self)
        self.PIN_15_8     = _IoMatrix_Pin158(self)
        self.PIN_18_16    = _IoMatrix_Pin1816(self)
        self.IO_PULL_UP   = _IoMatrix_IoPullUp(self)
        self.IO_PULL_DOWN = _IoMatrix_IoPullDown(self)


class _IoMatrix_Pin70(Register):
    ADDRESS = 0x48013000
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.ALTF0 = Field(self.ADDRESS, self.WIDTH, 0x0000000F,  0, "IOMATRIX.PIN_7_0.ALTF0", signed=False, parent=self)
        self.ALTF1 = Field(self.ADDRESS, self.WIDTH, 0x000000F0,  4, "IOMATRIX.PIN_7_0.ALTF1", signed=False, parent=self)
        self.ALTF2 = Field(self.ADDRESS, self.WIDTH, 0x00000F00,  8, "IOMATRIX.PIN_7_0.ALTF2", signed=False, parent=self)
        self.ALTF3 = Field(self.ADDRESS, self.WIDTH, 0x0000F000, 12, "IOMATRIX.PIN_7_0.ALTF3", signed=False, parent=self)
        self.ALTF4 = Field(self.ADDRESS, self.WIDTH, 0x000F0000, 16, "IOMATRIX.PIN_7_0.ALTF4", signed=False, parent=self)
        self.ALTF5 = Field(self.ADDRESS, self.WIDTH, 0x00F00000, 20, "IOMATRIX.PIN_7_0.ALTF5", signed=False, parent=self)
        self.ALTF6 = Field(self.ADDRESS, self.WIDTH, 0x0F000000, 24, "IOMATRIX.PIN_7_0.ALTF6", signed=False, parent=self)
        self.ALTF7 = Field(self.ADDRESS, self.WIDTH, 0xF0000000, 28, "IOMATRIX.PIN_7_0.ALTF7", signed=False, parent=self)

class _IoMatrix_Pin158(Register):
    ADDRESS = 0x48013004
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.ALTF8  = Field(self.ADDRESS, self.WIDTH, 0x0000000F,  0, "IOMATRIX.PIN_15_8.ALTF8", signed=False, parent=self)
        self.ALTF9  = Field(self.ADDRESS, self.WIDTH, 0x000000F0,  4, "IOMATRIX.PIN_15_8.ALTF9", signed=False, parent=self)
        self.ALTF10 = Field(self.ADDRESS, self.WIDTH, 0x00000F00,  8, "IOMATRIX.PIN_15_8.ALTF10", signed=False, parent=self)
        self.ALTF11 = Field(self.ADDRESS, self.WIDTH, 0x0000F000, 12, "IOMATRIX.PIN_15_8.ALTF11", signed=False, parent=self)
        self.ALTF12 = Field(self.ADDRESS, self.WIDTH, 0x000F0000, 16, "IOMATRIX.PIN_15_8.ALTF12", signed=False, parent=self)
        self.ALTF13 = Field(self.ADDRESS, self.WIDTH, 0x00F00000, 20, "IOMATRIX.PIN_15_8.ALTF13", signed=False, parent=self)
        self.ALTF14 = Field(self.ADDRESS, self.WIDTH, 0x0F000000, 24, "IOMATRIX.PIN_15_8.ALTF14", signed=False, parent=self)
        self.ALTF15 = Field(self.ADDRESS, self.WIDTH, 0xF0000000, 28, "IOMATRIX.PIN_15_8.ALTF15", signed=False, parent=self)

class _IoMatrix_Pin1816(Register):
    ADDRESS = 0x48013008
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.ALTF16 = Field(self.ADDRESS, self.WIDTH, 0x0000000F,  0, "IOMATRIX.PIN_18_16.ALTF16", signed=False, parent=self)
        self.ALTF17 = Field(self.ADDRESS, self.WIDTH, 0x000000F0,  4, "IOMATRIX.PIN_18_16.ALTF17", signed=False, parent=self)
        self.ALTF18 = Field(self.ADDRESS, self.WIDTH, 0x00000F00,  8, "IOMATRIX.PIN_18_16.ALTF18", signed=False, parent=self)

class _IoMatrix_IoPullUp(Register):
    ADDRESS = 0x4801300C
    WIDTH = 4
    DEFAULT = 0x0007FFC0

    def __init__(self, parent):
        self._PARENT = parent

        self.PUR_EN_PIN_0  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_0", signed=False, parent=self)
        self.PUR_EN_PIN_1  = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_1", signed=False, parent=self)
        self.PUR_EN_PIN_2  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_2", signed=False, parent=self)
        self.PUR_EN_PIN_3  = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_3", signed=False, parent=self)
        self.PUR_EN_PIN_4  = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_4", signed=False, parent=self)
        self.PUR_EN_PIN_5  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_5", signed=False, parent=self)
        self.PUR_EN_PIN_6  = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_6", signed=False, parent=self)
        self.PUR_EN_PIN_7  = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_7", signed=False, parent=self)
        self.PUR_EN_PIN_8  = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_8", signed=False, parent=self)
        self.PUR_EN_PIN_9  = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_9", signed=False, parent=self)
        self.PUR_EN_PIN_10 = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_10", signed=False, parent=self)
        self.PUR_EN_PIN_11 = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_11", signed=False, parent=self)
        self.PUR_EN_PIN_12 = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_12", signed=False, parent=self)
        self.PUR_EN_PIN_13 = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_13", signed=False, parent=self)
        self.PUR_EN_PIN_14 = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_14", signed=False, parent=self)
        self.PUR_EN_PIN_15 = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_15", signed=False, parent=self)
        self.PUR_EN_PIN_16 = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_16", signed=False, parent=self)
        self.PUR_EN_PIN_17 = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_17", signed=False, parent=self)
        self.PUR_EN_PIN_18 = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "IOMATRIX.IO_PULL_UP.PUR_EN_PIN_18", signed=False, parent=self)

class _IoMatrix_IoPullDown(Register):
    ADDRESS = 0x48013010
    WIDTH = 4
    DEFAULT = 0x00000003

    def __init__(self, parent):
        self._PARENT = parent

        self.PDR_EN_PIN_0  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_0", signed=False, parent=self)
        self.PDR_EN_PIN_1  = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_1", signed=False, parent=self)
        self.PDR_EN_PIN_2  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_2", signed=False, parent=self)
        self.PDR_EN_PIN_3  = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_3", signed=False, parent=self)
        self.PDR_EN_PIN_4  = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_4", signed=False, parent=self)
        self.PDR_EN_PIN_5  = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_5", signed=False, parent=self)
        self.PDR_EN_PIN_6  = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_6", signed=False, parent=self)
        self.PDR_EN_PIN_7  = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_7", signed=False, parent=self)
        self.PDR_EN_PIN_8  = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_8", signed=False, parent=self)
        self.PDR_EN_PIN_9  = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_9", signed=False, parent=self)
        self.PDR_EN_PIN_10 = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_10", signed=False, parent=self)
        self.PDR_EN_PIN_11 = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_11", signed=False, parent=self)
        self.PDR_EN_PIN_12 = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_12", signed=False, parent=self)
        self.PDR_EN_PIN_13 = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_13", signed=False, parent=self)
        self.PDR_EN_PIN_14 = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_14", signed=False, parent=self)
        self.PDR_EN_PIN_15 = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_15", signed=False, parent=self)
        self.PDR_EN_PIN_16 = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_16", signed=False, parent=self)
        self.PDR_EN_PIN_17 = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_17", signed=False, parent=self)
        self.PDR_EN_PIN_18 = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "IOMATRIX.IO_PULL_DOWN.PDR_EN_PIN_18", signed=False, parent=self)


class _TimBasic1(AddressBlock):
    _NAME       = "TIMBASIC1"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.COUNTER  = _TimBasic1_Counter(self)
        self.LIMIT    = _TimBasic1_Limit(self)
        self.COMPARE  = _TimBasic1_Compare(self)
        self.CONFIG   = _TimBasic1_Config(self)
        self.IRQ_CTRL = _TimBasic1_IrqCtrl(self)
        self.STATUS   = _TimBasic1_Status(self)


class _TimBasic1_Counter(Register):
    ADDRESS = 0x48015000
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COUNTER = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMBASIC1.COUNTER.COUNTER", signed=False, parent=self)

class _TimBasic1_Limit(Register):
    ADDRESS = 0x48015004
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.LIMIT = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMBASIC1.LIMIT.LIMIT", signed=False, parent=self)

class _TimBasic1_Compare(Register):
    ADDRESS = 0x48015008
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COMPARE = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMBASIC1.COMPARE.COMPARE", signed=False, parent=self)

class _TimBasic1_Config(Register):
    ADDRESS = 0x4801500C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.ENABLE     = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "TIMBASIC1.CONFIG.ENABLE", signed=False, parent=self)
        self.PWM_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "TIMBASIC1.CONFIG.PWM_ENABLE", signed=False, parent=self)
        self.TOGGLE     = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "TIMBASIC1.CONFIG.TOGGLE", signed=False, parent=self)

class _TimBasic1_IrqCtrl(Register):
    ADDRESS = 0x48015010
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.OVER_IRQ_EN      = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "TIMBASIC1.IRQ_CTRL.OVER_IRQ_EN", signed=False, parent=self)
        self.CMP_MATCH_IRQ_EN = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "TIMBASIC1.IRQ_CTRL.CMP_MATCH_IRQ_EN", signed=False, parent=self)
        self.STOP_IRQ_EN      = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "TIMBASIC1.IRQ_CTRL.STOP_IRQ_EN", signed=False, parent=self)

class _TimBasic1_Status(Register):
    ADDRESS = 0x48015014
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.OVER_FLAG  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "TIMBASIC1.STATUS.OVER_FLAG", signed=False, parent=self)
        self.MATCH_FLAG = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "TIMBASIC1.STATUS.MATCH_FLAG", signed=False, parent=self)
        self.STOP_FLAG  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "TIMBASIC1.STATUS.STOP_FLAG", signed=False, parent=self)


class _TimBasic2(AddressBlock):
    _NAME       = "TIMBASIC2"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.COUNTER  = _TimBasic2_Counter(self)
        self.LIMIT    = _TimBasic2_Limit(self)
        self.COMPARE  = _TimBasic2_Compare(self)
        self.CONFIG   = _TimBasic2_Config(self)
        self.IRQ_CTRL = _TimBasic2_IrqCtrl(self)
        self.STATUS   = _TimBasic2_Status(self)


class _TimBasic2_Counter(Register):
    ADDRESS = 0x48016000
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COUNTER = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMBASIC2.COUNTER.COUNTER", signed=False, parent=self)

class _TimBasic2_Limit(Register):
    ADDRESS = 0x48016004
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.LIMIT = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMBASIC2.LIMIT.LIMIT", signed=False, parent=self)

class _TimBasic2_Compare(Register):
    ADDRESS = 0x48016008
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COMPARE = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMBASIC2.COMPARE.COMPARE", signed=False, parent=self)

class _TimBasic2_Config(Register):
    ADDRESS = 0x4801600C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.ENABLE     = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "TIMBASIC2.CONFIG.ENABLE", signed=False, parent=self)
        self.PWM_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "TIMBASIC2.CONFIG.PWM_ENABLE", signed=False, parent=self)
        self.TOGGLE     = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "TIMBASIC2.CONFIG.TOGGLE", signed=False, parent=self)

class _TimBasic2_IrqCtrl(Register):
    ADDRESS = 0x48016010
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.OVER_IRQ_EN      = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "TIMBASIC2.IRQ_CTRL.OVER_IRQ_EN", signed=False, parent=self)
        self.CMP_MATCH_IRQ_EN = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "TIMBASIC2.IRQ_CTRL.CMP_MATCH_IRQ_EN", signed=False, parent=self)
        self.STOP_IRQ_EN      = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "TIMBASIC2.IRQ_CTRL.STOP_IRQ_EN", signed=False, parent=self)

class _TimBasic2_Status(Register):
    ADDRESS = 0x48016014
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.OVER_FLAG  = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "TIMBASIC2.STATUS.OVER_FLAG", signed=False, parent=self)
        self.MATCH_FLAG = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "TIMBASIC2.STATUS.MATCH_FLAG", signed=False, parent=self)
        self.STOP_FLAG  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "TIMBASIC2.STATUS.STOP_FLAG", signed=False, parent=self)


class _Wdt(AddressBlock):
    _NAME       = "WDT"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.MAX     = _Wdt_Max(self)
        self.ENABLE  = _Wdt_Enable(self)
        self.LOCK    = _Wdt_Lock(self)
        self.CLEAR   = _Wdt_Clear(self)
        self.MIN     = _Wdt_Min(self)
        self.STATUS  = _Wdt_Status(self)
        self.COUNTER = _Wdt_Counter(self)


class _Wdt_Max(Register):
    ADDRESS = 0x48017000
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.MAX = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "WDT.MAX.MAX", signed=False, parent=self)

class _Wdt_Enable(Register):
    ADDRESS = 0x48017004
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.EN = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "WDT.ENABLE.EN", signed=False, parent=self)

class _Wdt_Lock(Register):
    ADDRESS = 0x48017008
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.LOCK = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "WDT.LOCK.LOCK", signed=False, parent=self)

class _Wdt_Clear(Register):
    ADDRESS = 0x4801700C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.CLEAR = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "WDT.CLEAR.CLEAR", signed=False, parent=self)

class _Wdt_Min(Register):
    ADDRESS = 0x48017010
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.MIN    = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "WDT.MIN.MIN", signed=False, parent=self)
        self.MIN_EN = Field(self.ADDRESS, self.WIDTH, 0x80000000, 31, "WDT.MIN.MIN_EN", signed=False, parent=self)

class _Wdt_Status(Register):
    ADDRESS = 0x48017014
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RESET = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "WDT.STATUS.RESET", signed=False, parent=self)

class _Wdt_Counter(Register):
    ADDRESS = 0x48017018
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.CNT = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "WDT.COUNTER.CNT", signed=False, parent=self)


class _TimSys(AddressBlock):
    _NAME       = "TIMSYS"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.COUNTER_L   = _TimSys_CounterL(self)
        self.COUNTER_H   = _TimSys_CounterH(self)
        self.COMPARE_L   = _TimSys_CompareL(self)
        self.COMPARE_H   = _TimSys_CompareH(self)
        self.AUX_DIV     = _TimSys_AuxDiv(self)
        self.AUX_COUNTER = _TimSys_AuxCounter(self)
        self.AUX_COMPARE = _TimSys_AuxCompare(self)
        self.CONFIG      = _TimSys_Config(self)
        self.IRQ_CTRL    = _TimSys_IrqCtrl(self)
        self.STATUS      = _TimSys_Status(self)


class _TimSys_CounterL(Register):
    ADDRESS = 0x48018000
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COUNTER = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMSYS.COUNTER_L.COUNTER", signed=False, parent=self)

class _TimSys_CounterH(Register):
    ADDRESS = 0x48018004
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COUNTER = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMSYS.COUNTER_H.COUNTER", signed=False, parent=self)

class _TimSys_CompareL(Register):
    ADDRESS = 0x48018008
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COMPARE = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMSYS.COMPARE_L.COMPARE", signed=False, parent=self)

class _TimSys_CompareH(Register):
    ADDRESS = 0x4801800C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.COMPARE = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMSYS.COMPARE_H.COMPARE", signed=False, parent=self)

class _TimSys_AuxDiv(Register):
    ADDRESS = 0x48018010
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.AUX_DIV = Field(self.ADDRESS, self.WIDTH, 0x0000FFFF,  0, "TIMSYS.AUX_DIV.AUX_DIV", signed=False, parent=self)

class _TimSys_AuxCounter(Register):
    ADDRESS = 0x48018014
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.AUX_COUNTER = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMSYS.AUX_COUNTER.AUX_COUNTER", signed=False, parent=self)

class _TimSys_AuxCompare(Register):
    ADDRESS = 0x48018018
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.AUX_COMPARE = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "TIMSYS.AUX_COMPARE.AUX_COMPARE", signed=False, parent=self)

class _TimSys_Config(Register):
    ADDRESS = 0x4801801C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.DBG_MODE = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "TIMSYS.CONFIG.DBG_MODE", signed=False, parent=self)
        self.CMP_EQ   = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "TIMSYS.CONFIG.CMP_EQ", signed=False, parent=self)

class _TimSys_IrqCtrl(Register):
    ADDRESS = 0x48018020
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.CMP_IRQ_EN     = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "TIMSYS.IRQ_CTRL.CMP_IRQ_EN", signed=False, parent=self)
        self.AUX_CMP_IRQ_EN = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "TIMSYS.IRQ_CTRL.AUX_CMP_IRQ_EN", signed=False, parent=self)

class _TimSys_Status(Register):
    ADDRESS = 0x48018024
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.CMP_MATCH_FLAG     = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "TIMSYS.STATUS.CMP_MATCH_FLAG", signed=False, parent=self)
        self.AUX_CMP_MATCH_FLAG = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "TIMSYS.STATUS.AUX_CMP_MATCH_FLAG", signed=False, parent=self)
        self.CODE               = Field(self.ADDRESS, self.WIDTH, 0xE0000000, 29, "TIMSYS.STATUS.CODE", signed=False, parent=self)


class _Aon(AddressBlock):
    _NAME       = "AON"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.CONFIG    = _Aon_Config(self)
        self.SLEEP     = _Aon_Sleep(self)
        self.STATUS    = _Aon_Status(self)
        self.BIAS      = _Aon_Bias(self)
        self.FLT_STS   = _Aon_FltSts(self)
        self.FLT_INT   = _Aon_FltInt(self)
        self.FLT_ENA_F = _Aon_FltEnaF(self)
        self.BL_REG0   = _Aon_BlReg0(self)
        self.BL_REG1   = _Aon_BlReg1(self)
        self.APP_REG0  = _Aon_AppReg0(self)
        self.APP_REG1  = _Aon_AppReg1(self)
        self.GP_REG0   = _Aon_GpReg0(self)
        self.GP_REG1   = _Aon_GpReg1(self)
        self.GP_REG2   = _Aon_GpReg2(self)
        self.GP_REG3   = _Aon_GpReg3(self)
        self.GP_REG4   = _Aon_GpReg4(self)
        self.GP_REG5   = _Aon_GpReg5(self)
        self.GP_REG6   = _Aon_GpReg6(self)
        self.GP_REG7   = _Aon_GpReg7(self)
        self.GP_REG8   = _Aon_GpReg8(self)
        self.GP_REG9   = _Aon_GpReg9(self)
        self.GP_REG10  = _Aon_GpReg10(self)
        self.GP_REG11  = _Aon_GpReg11(self)
        self.OTP_REG0  = _Aon_OtpReg0(self)
        self.OTP_REG1  = _Aon_OtpReg1(self)
        self.TM_REG0   = _Aon_TmReg0(self)
        self.TM_REG1   = _Aon_TmReg1(self)


class _Aon_Config(Register):
    ADDRESS = 0x48019000
    WIDTH = 1
    DEFAULT = 0x20

    def __init__(self, parent):
        self._PARENT = parent

        self.GDRV_PD_DIS = Field(self.ADDRESS, self.WIDTH, 0x01,  0, "AON.CONFIG.GDRV_PD_DIS", signed=False, parent=self)
        self.WKP_TIM_DIV = Field(self.ADDRESS, self.WIDTH, 0x0E,  1, "AON.CONFIG.WKP_TIM_DIV", signed=False, parent=self)
        self.WKP_SOURCE  = Field(self.ADDRESS, self.WIDTH, 0x30,  4, "AON.CONFIG.WKP_SOURCE", signed=False, parent=self)
        self.RFU_R0_VAON = Field(self.ADDRESS, self.WIDTH, 0xC0,  6, "AON.CONFIG.RFU_R0_VAON", signed=False, parent=self)

class _Aon_Sleep(Register):
    ADDRESS = 0x48019004
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.SLEEP = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "AON.SLEEP.SLEEP", signed=False, parent=self)

class _Aon_Status(Register):
    ADDRESS = 0x48019008
    WIDTH = 1
    DEFAULT = 0x6C

    def __init__(self, parent):
        self._PARENT = parent

        self.WKP_TIM_OVFL = Field(self.ADDRESS, self.WIDTH, 0x01,  0, "AON.STATUS.WKP_TIM_OVFL", signed=False, parent=self)
        self.V_EN_OK      = Field(self.ADDRESS, self.WIDTH, 0x02,  1, "AON.STATUS.V_EN_OK", signed=False, parent=self)
        self.AONPOKB      = Field(self.ADDRESS, self.WIDTH, 0x04,  2, "AON.STATUS.AONPOKB", signed=False, parent=self)
        self.AONPORB      = Field(self.ADDRESS, self.WIDTH, 0x08,  3, "AON.STATUS.AONPORB", signed=False, parent=self)
        self.WKP_EVENT    = Field(self.ADDRESS, self.WIDTH, 0x10,  4, "AON.STATUS.WKP_EVENT", signed=False, parent=self)
        self.TOK          = Field(self.ADDRESS, self.WIDTH, 0x20,  5, "AON.STATUS.TOK", signed=False, parent=self)
        self.CK32K_OK     = Field(self.ADDRESS, self.WIDTH, 0x40,  6, "AON.STATUS.CK32K_OK", signed=False, parent=self)

class _Aon_Bias(Register):
    ADDRESS = 0x4801900C
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.EN_OSC  = Field(self.ADDRESS, self.WIDTH, 0x01,  0, "AON.BIAS.EN_OSC", signed=False, parent=self)
        self.EN_BIAS = Field(self.ADDRESS, self.WIDTH, 0x02,  1, "AON.BIAS.EN_BIAS", signed=False, parent=self)

class _Aon_FltSts(Register):
    ADDRESS = 0x48019010
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.LDOVDD_TSD  = Field(self.ADDRESS, self.WIDTH, 0x01,  0, "AON.FLT_STS.LDOVDD_TSD", signed=False, parent=self)
        self.LDOVDDA_TSD = Field(self.ADDRESS, self.WIDTH, 0x02,  1, "AON.FLT_STS.LDOVDDA_TSD", signed=False, parent=self)
        self.ANA_TSD     = Field(self.ADDRESS, self.WIDTH, 0x04,  2, "AON.FLT_STS.ANA_TSD", signed=False, parent=self)
        self.BUCK_TSD    = Field(self.ADDRESS, self.WIDTH, 0x08,  3, "AON.FLT_STS.BUCK_TSD", signed=False, parent=self)

class _Aon_FltInt(Register):
    ADDRESS = 0x48019014
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.LDOVDD_TSD_LTC  = Field(self.ADDRESS, self.WIDTH, 0x01,  0, "AON.FLT_INT.LDOVDD_TSD_LTC", signed=False, parent=self)
        self.LDOVDDA_TSD_LTC = Field(self.ADDRESS, self.WIDTH, 0x02,  1, "AON.FLT_INT.LDOVDDA_TSD_LTC", signed=False, parent=self)
        self.ANA_TSD_LTC     = Field(self.ADDRESS, self.WIDTH, 0x04,  2, "AON.FLT_INT.ANA_TSD_LTC", signed=False, parent=self)
        self.BUCK_TSD_LTC    = Field(self.ADDRESS, self.WIDTH, 0x08,  3, "AON.FLT_INT.BUCK_TSD_LTC", signed=False, parent=self)

class _Aon_FltEnaF(Register):
    ADDRESS = 0x48019018
    WIDTH = 1
    DEFAULT = 0x0F

    def __init__(self, parent):
        self._PARENT = parent

        self.LDOVDD_ENA_F   = Field(self.ADDRESS, self.WIDTH, 0x01,  0, "AON.FLT_ENA_F.LDOVDD_ENA_F", signed=False, parent=self)
        self.LDOVDDA_ENA_F  = Field(self.ADDRESS, self.WIDTH, 0x02,  1, "AON.FLT_ENA_F.LDOVDDA_ENA_F", signed=False, parent=self)
        self.ANA_TSD_ENA_F  = Field(self.ADDRESS, self.WIDTH, 0x04,  2, "AON.FLT_ENA_F.ANA_TSD_ENA_F", signed=False, parent=self)
        self.BUCK_TSD_ENA_F = Field(self.ADDRESS, self.WIDTH, 0x08,  3, "AON.FLT_ENA_F.BUCK_TSD_ENA_F", signed=False, parent=self)

class _Aon_BlReg0(Register):
    ADDRESS = 0x48019020
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.FAULT_CNT  = Field(self.ADDRESS, self.WIDTH, 0x03,  0, "AON.BL_REG0.FAULT_CNT", signed=False, parent=self)
        self.BOOT_CAUSE = Field(self.ADDRESS, self.WIDTH, 0x0C,  2, "AON.BL_REG0.BOOT_CAUSE", signed=False, parent=self)
        self.BL_STATE   = Field(self.ADDRESS, self.WIDTH, 0xF0,  4, "AON.BL_REG0.BL_STATE", signed=False, parent=self)

class _Aon_BlReg1(Register):
    ADDRESS = 0x48019024
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.BL_DATA1 = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "AON.BL_REG1.BL_DATA1", signed=False, parent=self)

class _Aon_AppReg0(Register):
    ADDRESS = 0x48019028
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.APP_DATA0 = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "AON.APP_REG0.APP_DATA0", signed=False, parent=self)

class _Aon_AppReg1(Register):
    ADDRESS = 0x4801902C
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.APP_DATA1 = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "AON.APP_REG1.APP_DATA1", signed=False, parent=self)

class _Aon_GpReg0(Register):
    ADDRESS = 0x48019030
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.DATA0 = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "AON.GP_REG0.DATA0", signed=False, parent=self)

class _Aon_GpReg1(Register):
    ADDRESS = 0x48019034
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.DATA1 = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "AON.GP_REG1.DATA1", signed=False, parent=self)

class _Aon_GpReg2(Register):
    ADDRESS = 0x48019038
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.DATA2 = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "AON.GP_REG2.DATA2", signed=False, parent=self)

class _Aon_GpReg3(Register):
    ADDRESS = 0x4801903C
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.DATA3 = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "AON.GP_REG3.DATA3", signed=False, parent=self)

class _Aon_GpReg4(Register):
    ADDRESS = 0x48019040
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.DATA4 = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "AON.GP_REG4.DATA4", signed=False, parent=self)

class _Aon_GpReg5(Register):
    ADDRESS = 0x48019044
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.DATA5 = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "AON.GP_REG5.DATA5", signed=False, parent=self)

class _Aon_GpReg6(Register):
    ADDRESS = 0x48019048
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.DATA6 = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "AON.GP_REG6.DATA6", signed=False, parent=self)

class _Aon_GpReg7(Register):
    ADDRESS = 0x4801904C
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.DATA7 = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "AON.GP_REG7.DATA7", signed=False, parent=self)

class _Aon_GpReg8(Register):
    ADDRESS = 0x48019050
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.DATA8 = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "AON.GP_REG8.DATA8", signed=False, parent=self)

class _Aon_GpReg9(Register):
    ADDRESS = 0x48019054
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.DATA9 = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "AON.GP_REG9.DATA9", signed=False, parent=self)

class _Aon_GpReg10(Register):
    ADDRESS = 0x48019058
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.DATA10 = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "AON.GP_REG10.DATA10", signed=False, parent=self)

class _Aon_GpReg11(Register):
    ADDRESS = 0x4801905C
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.DATA11 = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "AON.GP_REG11.DATA11", signed=False, parent=self)

class _Aon_OtpReg0(Register):
    ADDRESS = 0x48019060
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.IBIAS_AON_CODE = Field(self.ADDRESS, self.WIDTH, 0x3F,  0, "AON.OTP_REG0.IBIAS_AON_CODE", signed=False, parent=self)

class _Aon_OtpReg1(Register):
    ADDRESS = 0x48019064
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.FOSC32K_CODE = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "AON.OTP_REG1.FOSC32K_CODE", signed=False, parent=self)

class _Aon_TmReg0(Register):
    ADDRESS = 0x48019070
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.ZTAT_CAL_EN       = Field(self.ADDRESS, self.WIDTH, 0x01,  0, "AON.TM_REG0.ZTAT_CAL_EN", signed=False, parent=self)
        self.ZTAT_CAL_OS       = Field(self.ADDRESS, self.WIDTH, 0x02,  1, "AON.TM_REG0.ZTAT_CAL_OS", signed=False, parent=self)
        self.ZTAT_CAL_READ_RES = Field(self.ADDRESS, self.WIDTH, 0x04,  2, "AON.TM_REG0.ZTAT_CAL_READ_RES", signed=False, parent=self)
        self.ZTAT_CAL_RUN      = Field(self.ADDRESS, self.WIDTH, 0x08,  3, "AON.TM_REG0.ZTAT_CAL_RUN", signed=False, parent=self)
        self.FRC_POK           = Field(self.ADDRESS, self.WIDTH, 0x10,  4, "AON.TM_REG0.FRC_POK", signed=False, parent=self)
        self.DIS_OSC           = Field(self.ADDRESS, self.WIDTH, 0x20,  5, "AON.TM_REG0.DIS_OSC", signed=False, parent=self)
        self.TM_SPARE0         = Field(self.ADDRESS, self.WIDTH, 0xC0,  6, "AON.TM_REG0.TM_SPARE0", signed=False, parent=self)

class _Aon_TmReg1(Register):
    ADDRESS = 0x48019074
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.TM_AON    = Field(self.ADDRESS, self.WIDTH, 0x0F,  0, "AON.TM_REG1.TM_AON", signed=False, parent=self)
        self.TM_SPARE1 = Field(self.ADDRESS, self.WIDTH, 0xF0,  4, "AON.TM_REG1.TM_SPARE1", signed=False, parent=self)


class _ClkCtrl(AddressBlock):
    _NAME       = "CLKCTRL"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.PLL_FB_CFG = _ClkCtrl_PllFbCfg(self)
        self.SOURCE     = _ClkCtrl_Source(self)
        self.PLL_CFG    = _ClkCtrl_PllCfg(self)
        self.OPT        = _ClkCtrl_Opt(self)
        self.STATUS     = _ClkCtrl_Status(self)
        self.LATCH_RE   = _ClkCtrl_LatchRe(self)
        self.LATCH_FE   = _ClkCtrl_LatchFe(self)
        self.INT_ENA_RE = _ClkCtrl_IntEnaRe(self)
        self.INT_ENA_FE = _ClkCtrl_IntEnaFe(self)
        self.FLT_ENA_RE = _ClkCtrl_FltEnaRe(self)
        self.FLT_ENA_FE = _ClkCtrl_FltEnaFe(self)
        self.ERR_REPORT = _ClkCtrl_ErrReport(self)
        self.STS_REPORT = _ClkCtrl_StsReport(self)
        self.DET_CFG_0  = _ClkCtrl_DetCfg0(self)
        self.DET_CFG_1  = _ClkCtrl_DetCfg1(self)
        self.DET_CFG_2  = _ClkCtrl_DetCfg2(self)
        self.DET_CFG_3  = _ClkCtrl_DetCfg3(self)
        self.TM_0       = _ClkCtrl_Tm0(self)
        self.TM_1       = _ClkCtrl_Tm1(self)
        self.TM_2       = _ClkCtrl_Tm2(self)
        self.TM_3       = _ClkCtrl_Tm3(self)
        self.TM_4       = _ClkCtrl_Tm4(self)


class _ClkCtrl_PllFbCfg(Register):
    ADDRESS = 0x4801A000
    WIDTH = 1
    DEFAULT = 0x63

    def __init__(self, parent):
        self._PARENT = parent

        self.PLL_FB_DIV = Field(self.ADDRESS, self.WIDTH, 0x7F,  0, "CLKCTRL.PLL_FB_CFG.PLL_FB_DIV", signed=False, parent=self)

class _ClkCtrl_Source(Register):
    ADDRESS = 0x4801A004
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.EXT_NOT_XTAL = Field(self.ADDRESS, self.WIDTH, 0x01,  0, "CLKCTRL.SOURCE.EXT_NOT_XTAL", signed=False, parent=self)
        self.XTAL_CFG     = Field(self.ADDRESS, self.WIDTH, 0x0E,  1, "CLKCTRL.SOURCE.XTAL_CFG", signed=False, parent=self)
        self.XTAL_BOOST   = Field(self.ADDRESS, self.WIDTH, 0x10,  4, "CLKCTRL.SOURCE.XTAL_BOOST", signed=False, parent=self)
        self.EXT_NOT_INT  = Field(self.ADDRESS, self.WIDTH, 0x20,  5, "CLKCTRL.SOURCE.EXT_NOT_INT", signed=False, parent=self)
        self.BUCK_CK_SEL  = Field(self.ADDRESS, self.WIDTH, 0x80,  7, "CLKCTRL.SOURCE.BUCK_CK_SEL", signed=False, parent=self)

class _ClkCtrl_PllCfg(Register):
    ADDRESS = 0x4801A008
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.PLL_OUT_SEL = Field(self.ADDRESS, self.WIDTH, 0x03,  0, "CLKCTRL.PLL_CFG.PLL_OUT_SEL", signed=False, parent=self)
        self.RDIV        = Field(self.ADDRESS, self.WIDTH, 0x7C,  2, "CLKCTRL.PLL_CFG.RDIV", signed=False, parent=self)
        self.COMMIT      = Field(self.ADDRESS, self.WIDTH, 0x80,  7, "CLKCTRL.PLL_CFG.COMMIT", signed=False, parent=self)

class _ClkCtrl_Opt(Register):
    ADDRESS = 0x4801A00C
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.PWM_CLK_ENA = Field(self.ADDRESS, self.WIDTH, 0x01,  0, "CLKCTRL.OPT.PWM_CLK_ENA", signed=False, parent=self)
        self.ADC_CLK_ENA = Field(self.ADDRESS, self.WIDTH, 0x02,  1, "CLKCTRL.OPT.ADC_CLK_ENA", signed=False, parent=self)
        self.OTP_CLK_ENA = Field(self.ADDRESS, self.WIDTH, 0x04,  2, "CLKCTRL.OPT.OTP_CLK_ENA", signed=False, parent=self)
        self.SYS_CLK_DIV = Field(self.ADDRESS, self.WIDTH, 0x18,  3, "CLKCTRL.OPT.SYS_CLK_DIV", signed=False, parent=self)
        self.DMY_CLK_REG = Field(self.ADDRESS, self.WIDTH, 0x20,  5, "CLKCTRL.OPT.DMY_CLK_REG", signed=False, parent=self)
        self.CLK_FSM_ENA = Field(self.ADDRESS, self.WIDTH, 0x40,  6, "CLKCTRL.OPT.CLK_FSM_ENA", signed=False, parent=self)
        self.INT_FSM_ENA = Field(self.ADDRESS, self.WIDTH, 0x80,  7, "CLKCTRL.OPT.INT_FSM_ENA", signed=False, parent=self)

class _ClkCtrl_Status(Register):
    ADDRESS = 0x4801A030
    WIDTH = 1
    DEFAULT = 0x28

    def __init__(self, parent):
        self._PARENT = parent

        self.CLK_FAIL      = Field(self.ADDRESS, self.WIDTH, 0x01,  0, "CLKCTRL.STATUS.CLK_FAIL", signed=False, parent=self)
        self.PLL_LOCK      = Field(self.ADDRESS, self.WIDTH, 0x04,  2, "CLKCTRL.STATUS.PLL_LOCK", signed=False, parent=self)
        self.PLL_STUCK     = Field(self.ADDRESS, self.WIDTH, 0x08,  3, "CLKCTRL.STATUS.PLL_STUCK", signed=False, parent=self)
        self.CLK_FAULT_TMO = Field(self.ADDRESS, self.WIDTH, 0x10,  4, "CLKCTRL.STATUS.CLK_FAULT_TMO", signed=False, parent=self)
        self.CLK_FAULT     = Field(self.ADDRESS, self.WIDTH, 0x20,  5, "CLKCTRL.STATUS.CLK_FAULT", signed=False, parent=self)
        self.XTAL_RDY_TMO  = Field(self.ADDRESS, self.WIDTH, 0x40,  6, "CLKCTRL.STATUS.XTAL_RDY_TMO", signed=False, parent=self)
        self.XTAL_READY    = Field(self.ADDRESS, self.WIDTH, 0x80,  7, "CLKCTRL.STATUS.XTAL_READY", signed=False, parent=self)

class _ClkCtrl_LatchRe(Register):
    ADDRESS = 0x4801A034
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.CLK_FAIL_LTC      = Field(self.ADDRESS, self.WIDTH, 0x01,  0, "CLKCTRL.LATCH_RE.CLK_FAIL_LTC", signed=False, parent=self)
        self.PLL_ERR_LTC       = Field(self.ADDRESS, self.WIDTH, 0x02,  1, "CLKCTRL.LATCH_RE.PLL_ERR_LTC", signed=False, parent=self)
        self.PLL_LOCK_RE       = Field(self.ADDRESS, self.WIDTH, 0x04,  2, "CLKCTRL.LATCH_RE.PLL_LOCK_RE", signed=False, parent=self)
        self.PLL_STUCK_RESET   = Field(self.ADDRESS, self.WIDTH, 0x08,  3, "CLKCTRL.LATCH_RE.PLL_STUCK_RESET", signed=False, parent=self)
        self.CLK_FAULT_TMO_LTC = Field(self.ADDRESS, self.WIDTH, 0x10,  4, "CLKCTRL.LATCH_RE.CLK_FAULT_TMO_LTC", signed=False, parent=self)
        self.CLK_FAULT_RESET   = Field(self.ADDRESS, self.WIDTH, 0x20,  5, "CLKCTRL.LATCH_RE.CLK_FAULT_RESET", signed=False, parent=self)
        self.XTAL_RDY_TMO_LTC  = Field(self.ADDRESS, self.WIDTH, 0x40,  6, "CLKCTRL.LATCH_RE.XTAL_RDY_TMO_LTC", signed=False, parent=self)
        self.XTAL_RDY_RE_LTC   = Field(self.ADDRESS, self.WIDTH, 0x80,  7, "CLKCTRL.LATCH_RE.XTAL_RDY_RE_LTC", signed=False, parent=self)

class _ClkCtrl_LatchFe(Register):
    ADDRESS = 0x4801A038
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.PLL_LOCK_FE     = Field(self.ADDRESS, self.WIDTH, 0x04,  2, "CLKCTRL.LATCH_FE.PLL_LOCK_FE", signed=False, parent=self)
        self.CLK_OK          = Field(self.ADDRESS, self.WIDTH, 0x20,  5, "CLKCTRL.LATCH_FE.CLK_OK", signed=False, parent=self)
        self.XTAL_RDY_FE_LTC = Field(self.ADDRESS, self.WIDTH, 0x80,  7, "CLKCTRL.LATCH_FE.XTAL_RDY_FE_LTC", signed=False, parent=self)

class _ClkCtrl_IntEnaRe(Register):
    ADDRESS = 0x4801A040
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.CLK_FAIL_ENA_INT     = Field(self.ADDRESS, self.WIDTH, 0x01,  0, "CLKCTRL.INT_ENA_RE.CLK_FAIL_ENA_INT", signed=False, parent=self)
        self.PLL_ERR_ENA_INT      = Field(self.ADDRESS, self.WIDTH, 0x02,  1, "CLKCTRL.INT_ENA_RE.PLL_ERR_ENA_INT", signed=False, parent=self)
        self.PLL_LOCK_RE_ENA_INT  = Field(self.ADDRESS, self.WIDTH, 0x04,  2, "CLKCTRL.INT_ENA_RE.PLL_LOCK_RE_ENA_INT", signed=False, parent=self)
        self.PLL_STUCK_ENA_INT    = Field(self.ADDRESS, self.WIDTH, 0x08,  3, "CLKCTRL.INT_ENA_RE.PLL_STUCK_ENA_INT", signed=False, parent=self)
        self.CLK_FLT_TMO_ENA_INT  = Field(self.ADDRESS, self.WIDTH, 0x10,  4, "CLKCTRL.INT_ENA_RE.CLK_FLT_TMO_ENA_INT", signed=False, parent=self)
        self.CLK_FLT_ENA_INT      = Field(self.ADDRESS, self.WIDTH, 0x20,  5, "CLKCTRL.INT_ENA_RE.CLK_FLT_ENA_INT", signed=False, parent=self)
        self.XTAL_RDY_TMO_ENA_INT = Field(self.ADDRESS, self.WIDTH, 0x40,  6, "CLKCTRL.INT_ENA_RE.XTAL_RDY_TMO_ENA_INT", signed=False, parent=self)
        self.XTAL_RDY_RE_ENA_INT  = Field(self.ADDRESS, self.WIDTH, 0x80,  7, "CLKCTRL.INT_ENA_RE.XTAL_RDY_RE_ENA_INT", signed=False, parent=self)

class _ClkCtrl_IntEnaFe(Register):
    ADDRESS = 0x4801A044
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.PLL_LOCK_FE_ENA_INT = Field(self.ADDRESS, self.WIDTH, 0x04,  2, "CLKCTRL.INT_ENA_FE.PLL_LOCK_FE_ENA_INT", signed=False, parent=self)
        self.CLK_OK_ENA_INT      = Field(self.ADDRESS, self.WIDTH, 0x20,  5, "CLKCTRL.INT_ENA_FE.CLK_OK_ENA_INT", signed=False, parent=self)
        self.XTAL_RDY_FE_ENA_INT = Field(self.ADDRESS, self.WIDTH, 0x80,  7, "CLKCTRL.INT_ENA_FE.XTAL_RDY_FE_ENA_INT", signed=False, parent=self)

class _ClkCtrl_FltEnaRe(Register):
    ADDRESS = 0x4801A048
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.CLK_FAIL_ENA_FLT     = Field(self.ADDRESS, self.WIDTH, 0x01,  0, "CLKCTRL.FLT_ENA_RE.CLK_FAIL_ENA_FLT", signed=False, parent=self)
        self.PLL_ERR_ENA_FLT      = Field(self.ADDRESS, self.WIDTH, 0x02,  1, "CLKCTRL.FLT_ENA_RE.PLL_ERR_ENA_FLT", signed=False, parent=self)
        self.PLL_LOCK_RE_ENA_FLT  = Field(self.ADDRESS, self.WIDTH, 0x04,  2, "CLKCTRL.FLT_ENA_RE.PLL_LOCK_RE_ENA_FLT", signed=False, parent=self)
        self.PLL_STUCK_ENA_FLT    = Field(self.ADDRESS, self.WIDTH, 0x08,  3, "CLKCTRL.FLT_ENA_RE.PLL_STUCK_ENA_FLT", signed=False, parent=self)
        self.CLK_FLT_TMO_ENA_FLT  = Field(self.ADDRESS, self.WIDTH, 0x10,  4, "CLKCTRL.FLT_ENA_RE.CLK_FLT_TMO_ENA_FLT", signed=False, parent=self)
        self.CLK_FLT_ENA_FLT      = Field(self.ADDRESS, self.WIDTH, 0x20,  5, "CLKCTRL.FLT_ENA_RE.CLK_FLT_ENA_FLT", signed=False, parent=self)
        self.XTAL_RDY_TMO_ENA_FLT = Field(self.ADDRESS, self.WIDTH, 0x40,  6, "CLKCTRL.FLT_ENA_RE.XTAL_RDY_TMO_ENA_FLT", signed=False, parent=self)
        self.XTAL_RDY_RE_ENA_FLT  = Field(self.ADDRESS, self.WIDTH, 0x80,  7, "CLKCTRL.FLT_ENA_RE.XTAL_RDY_RE_ENA_FLT", signed=False, parent=self)

class _ClkCtrl_FltEnaFe(Register):
    ADDRESS = 0x4801A04C
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.PLL_LOCK_FE_ENA_FLT = Field(self.ADDRESS, self.WIDTH, 0x04,  2, "CLKCTRL.FLT_ENA_FE.PLL_LOCK_FE_ENA_FLT", signed=False, parent=self)
        self.CLK_OK_ENA_FLT      = Field(self.ADDRESS, self.WIDTH, 0x20,  5, "CLKCTRL.FLT_ENA_FE.CLK_OK_ENA_FLT", signed=False, parent=self)
        self.XTAL_RDY_FE_ENA_FLT = Field(self.ADDRESS, self.WIDTH, 0x80,  7, "CLKCTRL.FLT_ENA_FE.XTAL_RDY_FE_ENA_FLT", signed=False, parent=self)

class _ClkCtrl_ErrReport(Register):
    ADDRESS = 0x4801A050
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.CLK_FSM_ERR_RPT = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "CLKCTRL.ERR_REPORT.CLK_FSM_ERR_RPT", signed=False, parent=self)

class _ClkCtrl_StsReport(Register):
    ADDRESS = 0x4801A054
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.CLK_FSM_STS_RPT = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "CLKCTRL.STS_REPORT.CLK_FSM_STS_RPT", signed=False, parent=self)

class _ClkCtrl_DetCfg0(Register):
    ADDRESS = 0x4801A070
    WIDTH = 1
    DEFAULT = 0x0C

    def __init__(self, parent):
        self._PARENT = parent

        self.ONE_PER_MIN     = Field(self.ADDRESS, self.WIDTH, 0x1F,  0, "CLKCTRL.DET_CFG_0.ONE_PER_MIN", signed=False, parent=self)
        self.PLL_DEL_SEL     = Field(self.ADDRESS, self.WIDTH, 0x60,  5, "CLKCTRL.DET_CFG_0.PLL_DEL_SEL", signed=False, parent=self)
        self.N16_PER_MIN_MSB = Field(self.ADDRESS, self.WIDTH, 0x80,  7, "CLKCTRL.DET_CFG_0.N16_PER_MIN_MSB", signed=False, parent=self)

class _ClkCtrl_DetCfg1(Register):
    ADDRESS = 0x4801A074
    WIDTH = 1
    DEFAULT = 0x50

    def __init__(self, parent):
        self._PARENT = parent

        self.ONE_PER_MAX     = Field(self.ADDRESS, self.WIDTH, 0x1F,  0, "CLKCTRL.DET_CFG_1.ONE_PER_MAX", signed=False, parent=self)
        self.PLL_ERR_OTP     = Field(self.ADDRESS, self.WIDTH, 0x20,  5, "CLKCTRL.DET_CFG_1.PLL_ERR_OTP", signed=False, parent=self)
        self.SEL_ERR_OTP     = Field(self.ADDRESS, self.WIDTH, 0x40,  6, "CLKCTRL.DET_CFG_1.SEL_ERR_OTP", signed=False, parent=self)
        self.N16_PER_MAX_MSB = Field(self.ADDRESS, self.WIDTH, 0x80,  7, "CLKCTRL.DET_CFG_1.N16_PER_MAX_MSB", signed=False, parent=self)

class _ClkCtrl_DetCfg2(Register):
    ADDRESS = 0x4801A078
    WIDTH = 1
    DEFAULT = 0xDF

    def __init__(self, parent):
        self._PARENT = parent

        self.N16_PER_MIN = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "CLKCTRL.DET_CFG_2.N16_PER_MIN", signed=False, parent=self)

class _ClkCtrl_DetCfg3(Register):
    ADDRESS = 0x4801A07C
    WIDTH = 1
    DEFAULT = 0xFF

    def __init__(self, parent):
        self._PARENT = parent

        self.N16_PER_MAX = Field(self.ADDRESS, self.WIDTH, 0xFF,  0, "CLKCTRL.DET_CFG_3.N16_PER_MAX", signed=False, parent=self)

class _ClkCtrl_Tm0(Register):
    ADDRESS = 0x4801A080
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.TM__CLK_CFG = Field(self.ADDRESS, self.WIDTH, 0x01,  0, "CLKCTRL.TM_0.TM__CLK_CFG", signed=False, parent=self)
        self.TM__XTL_ENA = Field(self.ADDRESS, self.WIDTH, 0x06,  1, "CLKCTRL.TM_0.TM__XTL_ENA", signed=False, parent=self)
        self.TM__XTL_CFG = Field(self.ADDRESS, self.WIDTH, 0x08,  3, "CLKCTRL.TM_0.TM__XTL_CFG", signed=False, parent=self)
        self.TM__1M0__OK = Field(self.ADDRESS, self.WIDTH, 0x10,  4, "CLKCTRL.TM_0.TM__1M0__OK", signed=False, parent=self)
        self.TM__XTL_RDY = Field(self.ADDRESS, self.WIDTH, 0x20,  5, "CLKCTRL.TM_0.TM__XTL_RDY", signed=False, parent=self)
        self.TM__PLL_ERR = Field(self.ADDRESS, self.WIDTH, 0x40,  6, "CLKCTRL.TM_0.TM__PLL_ERR", signed=False, parent=self)
        self.TM__CPU_REQ = Field(self.ADDRESS, self.WIDTH, 0x80,  7, "CLKCTRL.TM_0.TM__CPU_REQ", signed=False, parent=self)

class _ClkCtrl_Tm1(Register):
    ADDRESS = 0x4801A084
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.TM__15M_MDE = Field(self.ADDRESS, self.WIDTH, 0x01,  0, "CLKCTRL.TM_1.TM__15M_MDE", signed=False, parent=self)
        self.TM__CLK_MUX = Field(self.ADDRESS, self.WIDTH, 0x02,  1, "CLKCTRL.TM_1.TM__CLK_MUX", signed=False, parent=self)
        self.TM__BLK_CFG = Field(self.ADDRESS, self.WIDTH, 0x7C,  2, "CLKCTRL.TM_1.TM__BLK_CFG", signed=False, parent=self)
        self.TM__BLK_ENA = Field(self.ADDRESS, self.WIDTH, 0x80,  7, "CLKCTRL.TM_1.TM__BLK_ENA", signed=False, parent=self)

class _ClkCtrl_Tm2(Register):
    ADDRESS = 0x4801A088
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.TM__GTD_CFG = Field(self.ADDRESS, self.WIDTH, 0x07,  0, "CLKCTRL.TM_2.TM__GTD_CFG", signed=False, parent=self)
        self.TM__GTD_ENA = Field(self.ADDRESS, self.WIDTH, 0x08,  3, "CLKCTRL.TM_2.TM__GTD_ENA", signed=False, parent=self)
        self.TM__DMY_REG = Field(self.ADDRESS, self.WIDTH, 0xF0,  4, "CLKCTRL.TM_2.TM__DMY_REG", signed=False, parent=self)

class _ClkCtrl_Tm3(Register):
    ADDRESS = 0x4801A08C
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.TM__ANA_1M3 = Field(self.ADDRESS, self.WIDTH, 0x01,  0, "CLKCTRL.TM_3.TM__ANA_1M3", signed=False, parent=self)
        self.TM__FLT_500 = Field(self.ADDRESS, self.WIDTH, 0x02,  1, "CLKCTRL.TM_3.TM__FLT_500", signed=False, parent=self)
        self.TM_DIS_CKWD = Field(self.ADDRESS, self.WIDTH, 0x04,  2, "CLKCTRL.TM_3.TM_DIS_CKWD", signed=False, parent=self)
        self.TM_DMUX_SEL = Field(self.ADDRESS, self.WIDTH, 0xF8,  3, "CLKCTRL.TM_3.TM_DMUX_SEL", signed=False, parent=self)

class _ClkCtrl_Tm4(Register):
    ADDRESS = 0x4801A090
    WIDTH = 1
    DEFAULT = 0x00

    def __init__(self, parent):
        self._PARENT = parent

        self.TM__PLL_BUS = Field(self.ADDRESS, self.WIDTH, 0x1F,  0, "CLKCTRL.TM_4.TM__PLL_BUS", signed=False, parent=self)
        self.TM__PLL_FRC = Field(self.ADDRESS, self.WIDTH, 0x20,  5, "CLKCTRL.TM_4.TM__PLL_FRC", signed=False, parent=self)
        self.TM__FRC_PMU = Field(self.ADDRESS, self.WIDTH, 0x40,  6, "CLKCTRL.TM_4.TM__FRC_PMU", signed=False, parent=self)
        self.TM__FRC_EXT = Field(self.ADDRESS, self.WIDTH, 0x80,  7, "CLKCTRL.TM_4.TM__FRC_EXT", signed=False, parent=self)


class _SysCtrl(AddressBlock):
    _NAME       = "SYSCTRL"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.SW_RESET        = _SysCtrl_SwReset(self)
        self.SW_RESET_STATUS = _SysCtrl_SwResetStatus(self)
        self.PMU_CTRL        = _SysCtrl_PmuCtrl(self)
        self.FAULT_STS       = _SysCtrl_FaultSts(self)
        self.FAULT_R_INT     = _SysCtrl_FaultRInt(self)
        self.FAULT_R_ENA_F   = _SysCtrl_FaultREnaF(self)
        self.FAULT_R_ENA_I   = _SysCtrl_FaultREnaI(self)
        self.FAULT_F_INT     = _SysCtrl_FaultFInt(self)
        self.FAULT_F_ENA_F   = _SysCtrl_FaultFEnaF(self)
        self.FAULT_F_ENA_I   = _SysCtrl_FaultFEnaI(self)
        self.FREQ_TRIM_VAON  = _SysCtrl_FreqTrimVaon(self)
        self.IZTAT_TRIM_VAON = _SysCtrl_IztatTrimVaon(self)
        self.SW_LVL_OTP      = _SysCtrl_SwLvlOtp(self)
        self.READBACK_OTP    = _SysCtrl_ReadbackOtp(self)
        self.ADC_CFG0        = _SysCtrl_AdcCfg0(self)
        self.ADC_CFG1        = _SysCtrl_AdcCfg1(self)
        self.ADC_CFG2        = _SysCtrl_AdcCfg2(self)
        self.ADC_CFG3        = _SysCtrl_AdcCfg3(self)
        self.ADC_CFG4        = _SysCtrl_AdcCfg4(self)
        self.ADC_CFG5        = _SysCtrl_AdcCfg5(self)
        self.ADC_CFG6        = _SysCtrl_AdcCfg6(self)
        self.ADC_CFG7        = _SysCtrl_AdcCfg7(self)
        self.ADC_CFG8        = _SysCtrl_AdcCfg8(self)


class _SysCtrl_SwReset(Register):
    ADDRESS = 0x4801B000
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RESET = Field(self.ADDRESS, self.WIDTH, 0x0000000F,  0, "SYSCTRL.SW_RESET.RESET", signed=False, parent=self)
        self.OTHER = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "SYSCTRL.SW_RESET.OTHER", signed=False, parent=self)
        self.CPU   = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "SYSCTRL.SW_RESET.CPU", signed=False, parent=self)

class _SysCtrl_SwResetStatus(Register):
    ADDRESS = 0x4801B004
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.OTHER = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "SYSCTRL.SW_RESET_STATUS.OTHER", signed=False, parent=self)
        self.CPU   = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "SYSCTRL.SW_RESET_STATUS.CPU", signed=False, parent=self)

class _SysCtrl_PmuCtrl(Register):
    ADDRESS = 0x4801B010
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.CP_ENABLE     = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "SYSCTRL.PMU_CTRL.CP_ENABLE", signed=False, parent=self)
        self.LDO_ENABLE    = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "SYSCTRL.PMU_CTRL.LDO_ENABLE", signed=False, parent=self)
        self.VEXT1_CNFG    = Field(self.ADDRESS, self.WIDTH, 0x0000000C,  2, "SYSCTRL.PMU_CTRL.VEXT1_CNFG", signed=False, parent=self)
        self.VEXT2_CNFG    = Field(self.ADDRESS, self.WIDTH, 0x00000030,  4, "SYSCTRL.PMU_CTRL.VEXT2_CNFG", signed=False, parent=self)
        self.VEXT1_SS_CNFG = Field(self.ADDRESS, self.WIDTH, 0x000000C0,  6, "SYSCTRL.PMU_CTRL.VEXT1_SS_CNFG", signed=False, parent=self)
        self.VEXT2_SS_CNFG = Field(self.ADDRESS, self.WIDTH, 0x00000300,  8, "SYSCTRL.PMU_CTRL.VEXT2_SS_CNFG", signed=False, parent=self)
        self.FRC_CP_SHTDW  = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "SYSCTRL.PMU_CTRL.FRC_CP_SHTDW", signed=False, parent=self)

class _SysCtrl_FaultSts(Register):
    ADDRESS = 0x4801B020
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.BCK_UVLO      = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "SYSCTRL.FAULT_STS.BCK_UVLO", signed=False, parent=self)
        self.BCK_SHORT     = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "SYSCTRL.FAULT_STS.BCK_SHORT", signed=False, parent=self)
        self.LDOEXT_TSD    = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "SYSCTRL.FAULT_STS.LDOEXT_TSD", signed=False, parent=self)
        self.LDOEXT1_SHORT = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "SYSCTRL.FAULT_STS.LDOEXT1_SHORT", signed=False, parent=self)
        self.LDOEXT2_SHORT = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "SYSCTRL.FAULT_STS.LDOEXT2_SHORT", signed=False, parent=self)
        self.CHGP_OK       = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "SYSCTRL.FAULT_STS.CHGP_OK", signed=False, parent=self)
        self.CHGP_SHORT    = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "SYSCTRL.FAULT_STS.CHGP_SHORT", signed=False, parent=self)
        self.VSA_UVLO      = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "SYSCTRL.FAULT_STS.VSA_UVLO", signed=False, parent=self)
        self.VDD_UVLO      = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "SYSCTRL.FAULT_STS.VDD_UVLO", signed=False, parent=self)
        self.VDDA_UVLO     = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "SYSCTRL.FAULT_STS.VDDA_UVLO", signed=False, parent=self)
        self.VCCIO_UVLO    = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "SYSCTRL.FAULT_STS.VCCIO_UVLO", signed=False, parent=self)
        self.LDO1_READY    = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "SYSCTRL.FAULT_STS.LDO1_READY", signed=False, parent=self)
        self.LDO2_READY    = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "SYSCTRL.FAULT_STS.LDO2_READY", signed=False, parent=self)
        self.V_EN_OK       = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "SYSCTRL.FAULT_STS.V_EN_OK", signed=False, parent=self)

class _SysCtrl_FaultRInt(Register):
    ADDRESS = 0x4801B024
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.BCK_UVLO_LTC      = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "SYSCTRL.FAULT_R_INT.BCK_UVLO_LTC", signed=False, parent=self)
        self.BCK_SHORT_RE_LTC  = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "SYSCTRL.FAULT_R_INT.BCK_SHORT_RE_LTC", signed=False, parent=self)
        self.LDOEXT_TSD_LTC    = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "SYSCTRL.FAULT_R_INT.LDOEXT_TSD_LTC", signed=False, parent=self)
        self.LDOEXT1_SHORT_LTC = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "SYSCTRL.FAULT_R_INT.LDOEXT1_SHORT_LTC", signed=False, parent=self)
        self.LDOEXT2_SHORT_LTC = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "SYSCTRL.FAULT_R_INT.LDOEXT2_SHORT_LTC", signed=False, parent=self)
        self.CHGP_OK_LTC       = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "SYSCTRL.FAULT_R_INT.CHGP_OK_LTC", signed=False, parent=self)
        self.CHGP_SHORT_LTC    = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "SYSCTRL.FAULT_R_INT.CHGP_SHORT_LTC", signed=False, parent=self)
        self.VSA_UVLO_LTC      = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "SYSCTRL.FAULT_R_INT.VSA_UVLO_LTC", signed=False, parent=self)
        self.VDD_UVLO_LTC      = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "SYSCTRL.FAULT_R_INT.VDD_UVLO_LTC", signed=False, parent=self)
        self.VDDA_UVLO_LTC     = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "SYSCTRL.FAULT_R_INT.VDDA_UVLO_LTC", signed=False, parent=self)
        self.VCCIO_UVLO_LTC    = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "SYSCTRL.FAULT_R_INT.VCCIO_UVLO_LTC", signed=False, parent=self)
        self.LDO1_READY_RE_LTC = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "SYSCTRL.FAULT_R_INT.LDO1_READY_RE_LTC", signed=False, parent=self)
        self.LDO2_READY_RE_LTC = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "SYSCTRL.FAULT_R_INT.LDO2_READY_RE_LTC", signed=False, parent=self)
        self.V_EN_OK_RE_LTC    = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "SYSCTRL.FAULT_R_INT.V_EN_OK_RE_LTC", signed=False, parent=self)
        self.UC_FAULT          = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "SYSCTRL.FAULT_R_INT.UC_FAULT", signed=False, parent=self)

class _SysCtrl_FaultREnaF(Register):
    ADDRESS = 0x4801B028
    WIDTH = 4
    DEFAULT = 0x00000783

    def __init__(self, parent):
        self._PARENT = parent

        self.BCK_UVLO_ENA_F      = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "SYSCTRL.FAULT_R_ENA_F.BCK_UVLO_ENA_F", signed=False, parent=self)
        self.BCK_SHORT_RE_ENA_F  = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "SYSCTRL.FAULT_R_ENA_F.BCK_SHORT_RE_ENA_F", signed=False, parent=self)
        self.LDOEXT_TSD_ENA_F    = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "SYSCTRL.FAULT_R_ENA_F.LDOEXT_TSD_ENA_F", signed=False, parent=self)
        self.LDOEXT1_SHORT_ENA_F = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "SYSCTRL.FAULT_R_ENA_F.LDOEXT1_SHORT_ENA_F", signed=False, parent=self)
        self.LDOEXT2_SHORT_ENA_F = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "SYSCTRL.FAULT_R_ENA_F.LDOEXT2_SHORT_ENA_F", signed=False, parent=self)
        self.CHGP_OK_ENA_F       = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "SYSCTRL.FAULT_R_ENA_F.CHGP_OK_ENA_F", signed=False, parent=self)
        self.CHGP_SHORT_ENA_F    = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "SYSCTRL.FAULT_R_ENA_F.CHGP_SHORT_ENA_F", signed=False, parent=self)
        self.VSA_UVLO_ENA_F      = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "SYSCTRL.FAULT_R_ENA_F.VSA_UVLO_ENA_F", signed=False, parent=self)
        self.VDD_UVLO_ENA_F      = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "SYSCTRL.FAULT_R_ENA_F.VDD_UVLO_ENA_F", signed=False, parent=self)
        self.VDDA_UVLO_ENA_F     = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "SYSCTRL.FAULT_R_ENA_F.VDDA_UVLO_ENA_F", signed=False, parent=self)
        self.VCCIO_UVLO_ENA_F    = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "SYSCTRL.FAULT_R_ENA_F.VCCIO_UVLO_ENA_F", signed=False, parent=self)
        self.LDO1_READY_RE_ENA_F = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "SYSCTRL.FAULT_R_ENA_F.LDO1_READY_RE_ENA_F", signed=False, parent=self)
        self.LDO2_READY_RE_ENA_F = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "SYSCTRL.FAULT_R_ENA_F.LDO2_READY_RE_ENA_F", signed=False, parent=self)
        self.V_EN_OK_RE_ENA_F    = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "SYSCTRL.FAULT_R_ENA_F.V_EN_OK_RE_ENA_F", signed=False, parent=self)

class _SysCtrl_FaultREnaI(Register):
    ADDRESS = 0x4801B02C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.LDOEXT_TSD_ENA_I    = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "SYSCTRL.FAULT_R_ENA_I.LDOEXT_TSD_ENA_I", signed=False, parent=self)
        self.LDOEXT1_SHORT_ENA_I = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "SYSCTRL.FAULT_R_ENA_I.LDOEXT1_SHORT_ENA_I", signed=False, parent=self)
        self.LDOEXT2_SHORT_ENA_I = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "SYSCTRL.FAULT_R_ENA_I.LDOEXT2_SHORT_ENA_I", signed=False, parent=self)
        self.CHGP_OK_ENA_I       = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "SYSCTRL.FAULT_R_ENA_I.CHGP_OK_ENA_I", signed=False, parent=self)
        self.CHGP_SHORT_ENA_I    = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "SYSCTRL.FAULT_R_ENA_I.CHGP_SHORT_ENA_I", signed=False, parent=self)
        self.LDO1_READY_RE_ENA_I = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "SYSCTRL.FAULT_R_ENA_I.LDO1_READY_RE_ENA_I", signed=False, parent=self)
        self.LDO2_READY_RE_ENA_I = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "SYSCTRL.FAULT_R_ENA_I.LDO2_READY_RE_ENA_I", signed=False, parent=self)
        self.V_EN_OK_RE_ENA_I    = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "SYSCTRL.FAULT_R_ENA_I.V_EN_OK_RE_ENA_I", signed=False, parent=self)

class _SysCtrl_FaultFInt(Register):
    ADDRESS = 0x4801B030
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.V_EN_OK_FE_LTC = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "SYSCTRL.FAULT_F_INT.V_EN_OK_FE_LTC", signed=False, parent=self)

class _SysCtrl_FaultFEnaF(Register):
    ADDRESS = 0x4801B034
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.V_EN_OK_FE_ENA_F = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "SYSCTRL.FAULT_F_ENA_F.V_EN_OK_FE_ENA_F", signed=False, parent=self)

class _SysCtrl_FaultFEnaI(Register):
    ADDRESS = 0x4801B038
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.V_EN_OK_FE_ENA_I = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "SYSCTRL.FAULT_F_ENA_I.V_EN_OK_FE_ENA_I", signed=False, parent=self)

class _SysCtrl_FreqTrimVaon(Register):
    ADDRESS = 0x4801B040
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.FREQ_TRIM_VAON = Field(self.ADDRESS, self.WIDTH, 0x000000FF,  0, "SYSCTRL.FREQ_TRIM_VAON.FREQ_TRIM_VAON", signed=False, parent=self)

class _SysCtrl_IztatTrimVaon(Register):
    ADDRESS = 0x4801B044
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.IZTAT_TRIM_VAON = Field(self.ADDRESS, self.WIDTH, 0x0000003F,  0, "SYSCTRL.IZTAT_TRIM_VAON.IZTAT_TRIM_VAON", signed=False, parent=self)

class _SysCtrl_SwLvlOtp(Register):
    ADDRESS = 0x4801B048
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.SW_LVL_OTP        = Field(self.ADDRESS, self.WIDTH, 0x00000007,  0, "SYSCTRL.SW_LVL_OTP.SW_LVL_OTP", signed=False, parent=self)
        self.BOOTLOADER_BYPASS = Field(self.ADDRESS, self.WIDTH, 0x000000F0,  4, "SYSCTRL.SW_LVL_OTP.BOOTLOADER_BYPASS", signed=False, parent=self)

class _SysCtrl_ReadbackOtp(Register):
    ADDRESS = 0x4801B04C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_READY     = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "SYSCTRL.READBACK_OTP.MTP_READY", signed=False, parent=self)
        self.MTP_READY_CLR = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "SYSCTRL.READBACK_OTP.MTP_READY_CLR", signed=False, parent=self)
        self.CP_READY_CLR  = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "SYSCTRL.READBACK_OTP.CP_READY_CLR", signed=False, parent=self)

class _SysCtrl_AdcCfg0(Register):
    ADDRESS = 0x4801B050
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.OFSL0 = Field(self.ADDRESS, self.WIDTH, 0x0000003F,  0, "SYSCTRL.ADC_CFG0.OFSL0", signed=False, parent=self)
        self.OFSM0 = Field(self.ADDRESS, self.WIDTH, 0x00000FC0,  6, "SYSCTRL.ADC_CFG0.OFSM0", signed=False, parent=self)
        self.OFSH0 = Field(self.ADDRESS, self.WIDTH, 0x0003F000, 12, "SYSCTRL.ADC_CFG0.OFSH0", signed=False, parent=self)

class _SysCtrl_AdcCfg1(Register):
    ADDRESS = 0x4801B054
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.OFSL1 = Field(self.ADDRESS, self.WIDTH, 0x0000003F,  0, "SYSCTRL.ADC_CFG1.OFSL1", signed=False, parent=self)
        self.OFSM1 = Field(self.ADDRESS, self.WIDTH, 0x00000FC0,  6, "SYSCTRL.ADC_CFG1.OFSM1", signed=False, parent=self)
        self.OFSH1 = Field(self.ADDRESS, self.WIDTH, 0x0003F000, 12, "SYSCTRL.ADC_CFG1.OFSH1", signed=False, parent=self)

class _SysCtrl_AdcCfg2(Register):
    ADDRESS = 0x4801B058
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.OFSL2 = Field(self.ADDRESS, self.WIDTH, 0x0000003F,  0, "SYSCTRL.ADC_CFG2.OFSL2", signed=False, parent=self)
        self.OFSM2 = Field(self.ADDRESS, self.WIDTH, 0x00000FC0,  6, "SYSCTRL.ADC_CFG2.OFSM2", signed=False, parent=self)
        self.OFSH2 = Field(self.ADDRESS, self.WIDTH, 0x0003F000, 12, "SYSCTRL.ADC_CFG2.OFSH2", signed=False, parent=self)

class _SysCtrl_AdcCfg3(Register):
    ADDRESS = 0x4801B05C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.OFSL3 = Field(self.ADDRESS, self.WIDTH, 0x0000003F,  0, "SYSCTRL.ADC_CFG3.OFSL3", signed=False, parent=self)
        self.OFSM3 = Field(self.ADDRESS, self.WIDTH, 0x00000FC0,  6, "SYSCTRL.ADC_CFG3.OFSM3", signed=False, parent=self)
        self.OFSH3 = Field(self.ADDRESS, self.WIDTH, 0x0003F000, 12, "SYSCTRL.ADC_CFG3.OFSH3", signed=False, parent=self)

class _SysCtrl_AdcCfg4(Register):
    ADDRESS = 0x4801B060
    WIDTH = 4
    DEFAULT = 0x08208200

    def __init__(self, parent):
        self._PARENT = parent

        self.SKIP_OFSH_AZ         = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "SYSCTRL.ADC_CFG4.SKIP_OFSH_AZ", signed=False, parent=self)
        self.SKIP_CMRN_AZ         = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "SYSCTRL.ADC_CFG4.SKIP_CMRN_AZ", signed=False, parent=self)
        self.SKIP_OFSL_AZ         = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "SYSCTRL.ADC_CFG4.SKIP_OFSL_AZ", signed=False, parent=self)
        self.SKIP_OFSM_AZ         = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "SYSCTRL.ADC_CFG4.SKIP_OFSM_AZ", signed=False, parent=self)
        self.CONFIG_VX2_TUNE_ADC0 = Field(self.ADDRESS, self.WIDTH, 0x000003F0,  4, "SYSCTRL.ADC_CFG4.CONFIG_VX2_TUNE_ADC0", signed=False, parent=self)
        self.CONFIG_VX2_TUNE_ADC1 = Field(self.ADDRESS, self.WIDTH, 0x0000FC00, 10, "SYSCTRL.ADC_CFG4.CONFIG_VX2_TUNE_ADC1", signed=False, parent=self)
        self.CONFIG_VX2_TUNE_ADC2 = Field(self.ADDRESS, self.WIDTH, 0x003F0000, 16, "SYSCTRL.ADC_CFG4.CONFIG_VX2_TUNE_ADC2", signed=False, parent=self)
        self.CONFIG_VX2_TUNE_ADC3 = Field(self.ADDRESS, self.WIDTH, 0x0FC00000, 22, "SYSCTRL.ADC_CFG4.CONFIG_VX2_TUNE_ADC3", signed=False, parent=self)

class _SysCtrl_AdcCfg5(Register):
    ADDRESS = 0x4801B064
    WIDTH = 4
    DEFAULT = 0x08080000

    def __init__(self, parent):
        self._PARENT = parent

        self.CMRN_DAC0 = Field(self.ADDRESS, self.WIDTH, 0x000FF000, 12, "SYSCTRL.ADC_CFG5.CMRN_DAC0", signed=False, parent=self)
        self.CMRP_DAC0 = Field(self.ADDRESS, self.WIDTH, 0x0FF00000, 20, "SYSCTRL.ADC_CFG5.CMRP_DAC0", signed=False, parent=self)

class _SysCtrl_AdcCfg6(Register):
    ADDRESS = 0x4801B068
    WIDTH = 4
    DEFAULT = 0x08080000

    def __init__(self, parent):
        self._PARENT = parent

        self.CMRN_DAC1 = Field(self.ADDRESS, self.WIDTH, 0x000FF000, 12, "SYSCTRL.ADC_CFG6.CMRN_DAC1", signed=False, parent=self)
        self.CMRP_DAC1 = Field(self.ADDRESS, self.WIDTH, 0x0FF00000, 20, "SYSCTRL.ADC_CFG6.CMRP_DAC1", signed=False, parent=self)

class _SysCtrl_AdcCfg7(Register):
    ADDRESS = 0x4801B06C
    WIDTH = 4
    DEFAULT = 0x08080000

    def __init__(self, parent):
        self._PARENT = parent

        self.CMRN_DAC2 = Field(self.ADDRESS, self.WIDTH, 0x000FF000, 12, "SYSCTRL.ADC_CFG7.CMRN_DAC2", signed=False, parent=self)
        self.CMRP_DAC2 = Field(self.ADDRESS, self.WIDTH, 0x0FF00000, 20, "SYSCTRL.ADC_CFG7.CMRP_DAC2", signed=False, parent=self)

class _SysCtrl_AdcCfg8(Register):
    ADDRESS = 0x4801B070
    WIDTH = 4
    DEFAULT = 0x08080000

    def __init__(self, parent):
        self._PARENT = parent

        self.CMRN_DAC3 = Field(self.ADDRESS, self.WIDTH, 0x000FF000, 12, "SYSCTRL.ADC_CFG8.CMRN_DAC3", signed=False, parent=self)
        self.CMRP_DAC3 = Field(self.ADDRESS, self.WIDTH, 0x0FF00000, 20, "SYSCTRL.ADC_CFG8.CMRP_DAC3", signed=False, parent=self)


class _Adc(AddressBlock):
    _NAME       = "ADC"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.RW_ADDR_DATA   = _Adc_RwAddrData(self)
        self.SRC_CONFIG     = _Adc_SrcConfig(self)
        self.SETUP          = _Adc_Setup(self)
        self.RD_DATA        = _Adc_RdData(self)
        self.VERSION        = _Adc_Version(self)
        self.STATUS         = _Adc_Status(self)
        self.CSA_AZ_VALS    = _Adc_CsaAzVals(self)
        self.CSA_SETUP      = _Adc_CsaSetup(self)
        self.CSA0_CONFIG    = _Adc_Csa0Config(self)
        self.CSA1_CONFIG    = _Adc_Csa1Config(self)
        self.CSA2_CONFIG    = _Adc_Csa2Config(self)
        self.CSA3_CONFIG    = _Adc_Csa3Config(self)
        self.CSA1_0_CMR_CFG = _Adc_Csa10CmrCfg(self)
        self.CSA3_2_CMR_CFG = _Adc_Csa32CmrCfg(self)


class _Adc_RwAddrData(Register):
    ADDRESS = 0x4801C000
    WIDTH = 4
    DEFAULT = 0xAA000000

    def __init__(self, parent):
        self._PARENT = parent

        self.REG_ADDR         = Field(self.ADDRESS, self.WIDTH, 0x0000003F,  0, "ADC.RW_ADDR_DATA.REG_ADDR", signed=False, parent=self)
        self.REG_DATA_WR      = Field(self.ADDRESS, self.WIDTH, 0x0000FF00,  8, "ADC.RW_ADDR_DATA.REG_DATA_WR", signed=False, parent=self)
        self.RD_STROBE        = Field(self.ADDRESS, self.WIDTH, 0x00010000, 16, "ADC.RW_ADDR_DATA.RD_STROBE", signed=False, parent=self)
        self.WR_STROBE        = Field(self.ADDRESS, self.WIDTH, 0x00020000, 17, "ADC.RW_ADDR_DATA.WR_STROBE", signed=False, parent=self)
        self.RD_DONE          = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "ADC.RW_ADDR_DATA.RD_DONE", signed=False, parent=self)
        self.WR_DONE          = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "ADC.RW_ADDR_DATA.WR_DONE", signed=False, parent=self)
        self.ADC0_CONFIG_DONE = Field(self.ADDRESS, self.WIDTH, 0x03000000, 24, "ADC.RW_ADDR_DATA.ADC0_CONFIG_DONE", signed=False, parent=self)
        self.ADC1_CONFIG_DONE = Field(self.ADDRESS, self.WIDTH, 0x0C000000, 26, "ADC.RW_ADDR_DATA.ADC1_CONFIG_DONE", signed=False, parent=self)
        self.ADC2_CONFIG_DONE = Field(self.ADDRESS, self.WIDTH, 0x30000000, 28, "ADC.RW_ADDR_DATA.ADC2_CONFIG_DONE", signed=False, parent=self)
        self.ADC3_CONFIG_DONE = Field(self.ADDRESS, self.WIDTH, 0xC0000000, 30, "ADC.RW_ADDR_DATA.ADC3_CONFIG_DONE", signed=False, parent=self)

class _Adc_SrcConfig(Register):
    ADDRESS = 0x4801C004
    WIDTH = 4
    DEFAULT = 0xB9B9B9B9

    def __init__(self, parent):
        self._PARENT = parent

        self.ADC0_MUX0_CFG    = Field(self.ADDRESS, self.WIDTH, 0x00000003,  0, "ADC.SRC_CONFIG.ADC0_MUX0_CFG", signed=False, parent=self)
        self.ADC0_MUX1_CFG    = Field(self.ADDRESS, self.WIDTH, 0x0000000C,  2, "ADC.SRC_CONFIG.ADC0_MUX1_CFG", signed=False, parent=self)
        self.ADC0_MUX2_CFG    = Field(self.ADDRESS, self.WIDTH, 0x00000030,  4, "ADC.SRC_CONFIG.ADC0_MUX2_CFG", signed=False, parent=self)
        self.ADC0_MUX3_DIS    = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "ADC.SRC_CONFIG.ADC0_MUX3_DIS", signed=False, parent=self)
        self.ADC0_MUX2_DETOUR = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "ADC.SRC_CONFIG.ADC0_MUX2_DETOUR", signed=False, parent=self)
        self.ADC1_MUX0_CFG    = Field(self.ADDRESS, self.WIDTH, 0x00000300,  8, "ADC.SRC_CONFIG.ADC1_MUX0_CFG", signed=False, parent=self)
        self.ADC1_MUX1_CFG    = Field(self.ADDRESS, self.WIDTH, 0x00000C00, 10, "ADC.SRC_CONFIG.ADC1_MUX1_CFG", signed=False, parent=self)
        self.ADC1_MUX2_CFG    = Field(self.ADDRESS, self.WIDTH, 0x00003000, 12, "ADC.SRC_CONFIG.ADC1_MUX2_CFG", signed=False, parent=self)
        self.ADC1_MUX3_DIS    = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "ADC.SRC_CONFIG.ADC1_MUX3_DIS", signed=False, parent=self)
        self.ADC1_MUX2_DETOUR = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "ADC.SRC_CONFIG.ADC1_MUX2_DETOUR", signed=False, parent=self)
        self.ADC2_MUX0_CFG    = Field(self.ADDRESS, self.WIDTH, 0x00030000, 16, "ADC.SRC_CONFIG.ADC2_MUX0_CFG", signed=False, parent=self)
        self.ADC2_MUX1_CFG    = Field(self.ADDRESS, self.WIDTH, 0x000C0000, 18, "ADC.SRC_CONFIG.ADC2_MUX1_CFG", signed=False, parent=self)
        self.ADC2_MUX2_CFG    = Field(self.ADDRESS, self.WIDTH, 0x00300000, 20, "ADC.SRC_CONFIG.ADC2_MUX2_CFG", signed=False, parent=self)
        self.ADC2_MUX3_DIS    = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "ADC.SRC_CONFIG.ADC2_MUX3_DIS", signed=False, parent=self)
        self.ADC2_MUX2_DETOUR = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "ADC.SRC_CONFIG.ADC2_MUX2_DETOUR", signed=False, parent=self)
        self.ADC3_MUX0_CFG    = Field(self.ADDRESS, self.WIDTH, 0x03000000, 24, "ADC.SRC_CONFIG.ADC3_MUX0_CFG", signed=False, parent=self)
        self.ADC3_MUX1_CFG    = Field(self.ADDRESS, self.WIDTH, 0x0C000000, 26, "ADC.SRC_CONFIG.ADC3_MUX1_CFG", signed=False, parent=self)
        self.ADC3_MUX2_CFG    = Field(self.ADDRESS, self.WIDTH, 0x30000000, 28, "ADC.SRC_CONFIG.ADC3_MUX2_CFG", signed=False, parent=self)
        self.ADC3_MUX3_DIS    = Field(self.ADDRESS, self.WIDTH, 0x40000000, 30, "ADC.SRC_CONFIG.ADC3_MUX3_DIS", signed=False, parent=self)
        self.ADC3_MUX2_DETOUR = Field(self.ADDRESS, self.WIDTH, 0x80000000, 31, "ADC.SRC_CONFIG.ADC3_MUX2_DETOUR", signed=False, parent=self)

class _Adc_Setup(Register):
    ADDRESS = 0x4801C008
    WIDTH = 4
    DEFAULT = 0x00F0BE0F

    def __init__(self, parent):
        self._PARENT = parent

        self.SELECT_ADC             = Field(self.ADDRESS, self.WIDTH, 0x0000000F,  0, "ADC.SETUP.SELECT_ADC", signed=False, parent=self)
        self.NRST_ADC_0             = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "ADC.SETUP.NRST_ADC_0", signed=False, parent=self)
        self.NRST_ADC_1             = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "ADC.SETUP.NRST_ADC_1", signed=False, parent=self)
        self.NRST_ADC_2             = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "ADC.SETUP.NRST_ADC_2", signed=False, parent=self)
        self.NRST_ADC_3             = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "ADC.SETUP.NRST_ADC_3", signed=False, parent=self)
        self.MUX0_AUTO_CHOP_DISABLE = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "ADC.SETUP.MUX0_AUTO_CHOP_DISABLE", signed=False, parent=self)
        self.MUX1_CHOP_ENABLE       = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "ADC.SETUP.MUX1_CHOP_ENABLE", signed=False, parent=self)
        self.MUX2_CHOP_ENABLE       = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "ADC.SETUP.MUX2_CHOP_ENABLE", signed=False, parent=self)
        self.MUX3_CHOP_ENABLE       = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "ADC.SETUP.MUX3_CHOP_ENABLE", signed=False, parent=self)
        self.AZ23CM_CHOP_ENABLE     = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "ADC.SETUP.AZ23CM_CHOP_ENABLE", signed=False, parent=self)
        self.AZ_CHOP_ENABLE         = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "ADC.SETUP.AZ_CHOP_ENABLE", signed=False, parent=self)
        self.EXT_ADC_AUTO_PROT      = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "ADC.SETUP.EXT_ADC_AUTO_PROT", signed=False, parent=self)
        self.MMU_ENABLE             = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "ADC.SETUP.MMU_ENABLE", signed=False, parent=self)
        self.SHIFT_SAMPLE           = Field(self.ADDRESS, self.WIDTH, 0x000F0000, 16, "ADC.SETUP.SHIFT_SAMPLE", signed=False, parent=self)
        self.ADC4_OV_MSKI           = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "ADC.SETUP.ADC4_OV_MSKI", signed=False, parent=self)
        self.ADC5_OV_MSKI           = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "ADC.SETUP.ADC5_OV_MSKI", signed=False, parent=self)
        self.ADC6_OV_MSKI           = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "ADC.SETUP.ADC6_OV_MSKI", signed=False, parent=self)
        self.ADC7_OV_MSKI           = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "ADC.SETUP.ADC7_OV_MSKI", signed=False, parent=self)
        self.HALF_WAKEUP_FREQ       = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "ADC.SETUP.HALF_WAKEUP_FREQ", signed=False, parent=self)

class _Adc_RdData(Register):
    ADDRESS = 0x4801C00C
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.DATA_ADC_0 = Field(self.ADDRESS, self.WIDTH, 0x000000FF,  0, "ADC.RD_DATA.DATA_ADC_0", signed=False, parent=self)
        self.DATA_ADC_1 = Field(self.ADDRESS, self.WIDTH, 0x0000FF00,  8, "ADC.RD_DATA.DATA_ADC_1", signed=False, parent=self)
        self.DATA_ADC_2 = Field(self.ADDRESS, self.WIDTH, 0x00FF0000, 16, "ADC.RD_DATA.DATA_ADC_2", signed=False, parent=self)
        self.DATA_ADC_3 = Field(self.ADDRESS, self.WIDTH, 0xFF000000, 24, "ADC.RD_DATA.DATA_ADC_3", signed=False, parent=self)

class _Adc_Version(Register):
    ADDRESS = 0x4801C010
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.VERSION_NUMBER = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "ADC.VERSION.VERSION_NUMBER", signed=False, parent=self)

class _Adc_Status(Register):
    ADDRESS = 0x4801C014
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.RDY_ADC_0        = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "ADC.STATUS.RDY_ADC_0", signed=False, parent=self)
        self.RDY_ADC_1        = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "ADC.STATUS.RDY_ADC_1", signed=False, parent=self)
        self.RDY_ADC_2        = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "ADC.STATUS.RDY_ADC_2", signed=False, parent=self)
        self.RDY_ADC_3        = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "ADC.STATUS.RDY_ADC_3", signed=False, parent=self)
        self.ADC4_OV_FLAG     = Field(self.ADDRESS, self.WIDTH, 0x00000010,  4, "ADC.STATUS.ADC4_OV_FLAG", signed=False, parent=self)
        self.ADC5_OV_FLAG     = Field(self.ADDRESS, self.WIDTH, 0x00000020,  5, "ADC.STATUS.ADC5_OV_FLAG", signed=False, parent=self)
        self.ADC6_OV_FLAG     = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "ADC.STATUS.ADC6_OV_FLAG", signed=False, parent=self)
        self.ADC7_OV_FLAG     = Field(self.ADDRESS, self.WIDTH, 0x00000080,  7, "ADC.STATUS.ADC7_OV_FLAG", signed=False, parent=self)
        self.ADC0_WTCHDG_FAIL = Field(self.ADDRESS, self.WIDTH, 0x00000100,  8, "ADC.STATUS.ADC0_WTCHDG_FAIL", signed=False, parent=self)
        self.ADC1_WTCHDG_FAIL = Field(self.ADDRESS, self.WIDTH, 0x00000200,  9, "ADC.STATUS.ADC1_WTCHDG_FAIL", signed=False, parent=self)
        self.ADC2_WTCHDG_FAIL = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "ADC.STATUS.ADC2_WTCHDG_FAIL", signed=False, parent=self)
        self.ADC3_WTCHDG_FAIL = Field(self.ADDRESS, self.WIDTH, 0x00000800, 11, "ADC.STATUS.ADC3_WTCHDG_FAIL", signed=False, parent=self)
        self.ADC0_MUXSEQ_FAIL = Field(self.ADDRESS, self.WIDTH, 0x00001000, 12, "ADC.STATUS.ADC0_MUXSEQ_FAIL", signed=False, parent=self)
        self.ADC1_MUXSEQ_FAIL = Field(self.ADDRESS, self.WIDTH, 0x00002000, 13, "ADC.STATUS.ADC1_MUXSEQ_FAIL", signed=False, parent=self)
        self.ADC2_MUXSEQ_FAIL = Field(self.ADDRESS, self.WIDTH, 0x00004000, 14, "ADC.STATUS.ADC2_MUXSEQ_FAIL", signed=False, parent=self)
        self.ADC3_MUXSEQ_FAIL = Field(self.ADDRESS, self.WIDTH, 0x00008000, 15, "ADC.STATUS.ADC3_MUXSEQ_FAIL", signed=False, parent=self)

class _Adc_CsaAzVals(Register):
    ADDRESS = 0x4801C018
    WIDTH = 4
    DEFAULT = 0x00000000

    def __init__(self, parent):
        self._PARENT = parent

        self.CSA_AZ_OFS_VALUES = Field(self.ADDRESS, self.WIDTH, 0xFFFFFFFF,  0, "ADC.CSA_AZ_VALS.CSA_AZ_OFS_VALUES", signed=False, parent=self)

class _Adc_CsaSetup(Register):
    ADDRESS = 0x4801C01C
    WIDTH = 4
    DEFAULT = 0x20000000

    def __init__(self, parent):
        self._PARENT = parent

        self.CSA0_EN             = Field(self.ADDRESS, self.WIDTH, 0x00000001,  0, "ADC.CSA_SETUP.CSA0_EN", signed=False, parent=self)
        self.CSA1_EN             = Field(self.ADDRESS, self.WIDTH, 0x00000002,  1, "ADC.CSA_SETUP.CSA1_EN", signed=False, parent=self)
        self.CSA2_EN             = Field(self.ADDRESS, self.WIDTH, 0x00000004,  2, "ADC.CSA_SETUP.CSA2_EN", signed=False, parent=self)
        self.CSA3_EN             = Field(self.ADDRESS, self.WIDTH, 0x00000008,  3, "ADC.CSA_SETUP.CSA3_EN", signed=False, parent=self)
        self.CSA012_GAIN         = Field(self.ADDRESS, self.WIDTH, 0x00000030,  4, "ADC.CSA_SETUP.CSA012_GAIN", signed=False, parent=self)
        self.CSA012_BYPASS       = Field(self.ADDRESS, self.WIDTH, 0x00000040,  6, "ADC.CSA_SETUP.CSA012_BYPASS", signed=False, parent=self)
        self.CSA3_GAIN           = Field(self.ADDRESS, self.WIDTH, 0x00000300,  8, "ADC.CSA_SETUP.CSA3_GAIN", signed=False, parent=self)
        self.CSA3_BYPASS         = Field(self.ADDRESS, self.WIDTH, 0x00000400, 10, "ADC.CSA_SETUP.CSA3_BYPASS", signed=False, parent=self)
        self.CSA012_FILT         = Field(self.ADDRESS, self.WIDTH, 0x00003000, 12, "ADC.CSA_SETUP.CSA012_FILT", signed=False, parent=self)
        self.CSA3_FILT           = Field(self.ADDRESS, self.WIDTH, 0x0000C000, 14, "ADC.CSA_SETUP.CSA3_FILT", signed=False, parent=self)
        self.CSA_AZ_FLTLNGTH_EXP = Field(self.ADDRESS, self.WIDTH, 0x000F0000, 16, "ADC.CSA_SETUP.CSA_AZ_FLTLNGTH_EXP", signed=False, parent=self)
        self.CSA_SLCT_AZ_VALS    = Field(self.ADDRESS, self.WIDTH, 0x00300000, 20, "ADC.CSA_SETUP.CSA_SLCT_AZ_VALS", signed=False, parent=self)
        self.AZ_SLCT_AZ_VALS     = Field(self.ADDRESS, self.WIDTH, 0x00C00000, 22, "ADC.CSA_SETUP.AZ_SLCT_AZ_VALS", signed=False, parent=self)
        self.CSA_AZ_TIME         = Field(self.ADDRESS, self.WIDTH, 0x07000000, 24, "ADC.CSA_SETUP.CSA_AZ_TIME", signed=False, parent=self)
        self.CSA_ADC_TRIG_BLK    = Field(self.ADDRESS, self.WIDTH, 0x70000000, 28, "ADC.CSA_SETUP.CSA_ADC_TRIG_BLK", signed=False, parent=self)

class _Adc_Csa0Config(Register):
    ADDRESS = 0x4801C020
    WIDTH = 4
    DEFAULT = 0x00C80000

    def __init__(self, parent):
        self._PARENT = parent

        self.OFSL0          = Field(self.ADDRESS, self.WIDTH, 0x0000003F,  0, "ADC.CSA0_CONFIG.OFSL0", signed=False, parent=self)
        self.OFSM0          = Field(self.ADDRESS, self.WIDTH, 0x00000FC0,  6, "ADC.CSA0_CONFIG.OFSM0", signed=False, parent=self)
        self.OFSH0          = Field(self.ADDRESS, self.WIDTH, 0x0003F000, 12, "ADC.CSA0_CONFIG.OFSH0", signed=False, parent=self)
        self.OFS_READY0     = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "ADC.CSA0_CONFIG.OFS_READY0", signed=False, parent=self)
        self.OFS_READY_FRC0 = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "ADC.CSA0_CONFIG.OFS_READY_FRC0", signed=False, parent=self)
        self.SKIP_CMR0      = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "ADC.CSA0_CONFIG.SKIP_CMR0", signed=False, parent=self)
        self.SKIP_OFS0      = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "ADC.CSA0_CONFIG.SKIP_OFS0", signed=False, parent=self)
        self.OFSCSA_EN0     = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "ADC.CSA0_CONFIG.OFSCSA_EN0", signed=False, parent=self)
        self.OFSADC_EN0     = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "ADC.CSA0_CONFIG.OFSADC_EN0", signed=False, parent=self)
        self.AZ_0           = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "ADC.CSA0_CONFIG.AZ_0", signed=False, parent=self)
        self.AZ2_0          = Field(self.ADDRESS, self.WIDTH, 0x02000000, 25, "ADC.CSA0_CONFIG.AZ2_0", signed=False, parent=self)
        self.AZ3_0          = Field(self.ADDRESS, self.WIDTH, 0x04000000, 26, "ADC.CSA0_CONFIG.AZ3_0", signed=False, parent=self)
        self.AZCM_0         = Field(self.ADDRESS, self.WIDTH, 0x08000000, 27, "ADC.CSA0_CONFIG.AZCM_0", signed=False, parent=self)
        self.CALIB_EN0      = Field(self.ADDRESS, self.WIDTH, 0x10000000, 28, "ADC.CSA0_CONFIG.CALIB_EN0", signed=False, parent=self)
        self.NVALID0        = Field(self.ADDRESS, self.WIDTH, 0x40000000, 30, "ADC.CSA0_CONFIG.NVALID0", signed=False, parent=self)
        self.TRIG0          = Field(self.ADDRESS, self.WIDTH, 0x80000000, 31, "ADC.CSA0_CONFIG.TRIG0", signed=False, parent=self)

class _Adc_Csa1Config(Register):
    ADDRESS = 0x4801C024
    WIDTH = 4
    DEFAULT = 0x00C80000

    def __init__(self, parent):
        self._PARENT = parent

        self.OFSL1          = Field(self.ADDRESS, self.WIDTH, 0x0000003F,  0, "ADC.CSA1_CONFIG.OFSL1", signed=False, parent=self)
        self.OFSM1          = Field(self.ADDRESS, self.WIDTH, 0x00000FC0,  6, "ADC.CSA1_CONFIG.OFSM1", signed=False, parent=self)
        self.OFSH1          = Field(self.ADDRESS, self.WIDTH, 0x0003F000, 12, "ADC.CSA1_CONFIG.OFSH1", signed=False, parent=self)
        self.OFS_READY1     = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "ADC.CSA1_CONFIG.OFS_READY1", signed=False, parent=self)
        self.OFS_READY_FRC1 = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "ADC.CSA1_CONFIG.OFS_READY_FRC1", signed=False, parent=self)
        self.SKIP_CMR1      = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "ADC.CSA1_CONFIG.SKIP_CMR1", signed=False, parent=self)
        self.SKIP_OFS1      = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "ADC.CSA1_CONFIG.SKIP_OFS1", signed=False, parent=self)
        self.OFSCSA_EN1     = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "ADC.CSA1_CONFIG.OFSCSA_EN1", signed=False, parent=self)
        self.OFSADC_EN1     = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "ADC.CSA1_CONFIG.OFSADC_EN1", signed=False, parent=self)
        self.AZ_1           = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "ADC.CSA1_CONFIG.AZ_1", signed=False, parent=self)
        self.AZ2_1          = Field(self.ADDRESS, self.WIDTH, 0x02000000, 25, "ADC.CSA1_CONFIG.AZ2_1", signed=False, parent=self)
        self.AZ3_1          = Field(self.ADDRESS, self.WIDTH, 0x04000000, 26, "ADC.CSA1_CONFIG.AZ3_1", signed=False, parent=self)
        self.AZCM_1         = Field(self.ADDRESS, self.WIDTH, 0x08000000, 27, "ADC.CSA1_CONFIG.AZCM_1", signed=False, parent=self)
        self.CALIB_EN1      = Field(self.ADDRESS, self.WIDTH, 0x10000000, 28, "ADC.CSA1_CONFIG.CALIB_EN1", signed=False, parent=self)
        self.NVALID1        = Field(self.ADDRESS, self.WIDTH, 0x40000000, 30, "ADC.CSA1_CONFIG.NVALID1", signed=False, parent=self)
        self.TRIG1          = Field(self.ADDRESS, self.WIDTH, 0x80000000, 31, "ADC.CSA1_CONFIG.TRIG1", signed=False, parent=self)

class _Adc_Csa2Config(Register):
    ADDRESS = 0x4801C028
    WIDTH = 4
    DEFAULT = 0x00C80000

    def __init__(self, parent):
        self._PARENT = parent

        self.OFSL2          = Field(self.ADDRESS, self.WIDTH, 0x0000003F,  0, "ADC.CSA2_CONFIG.OFSL2", signed=False, parent=self)
        self.OFSM2          = Field(self.ADDRESS, self.WIDTH, 0x00000FC0,  6, "ADC.CSA2_CONFIG.OFSM2", signed=False, parent=self)
        self.OFSH2          = Field(self.ADDRESS, self.WIDTH, 0x0003F000, 12, "ADC.CSA2_CONFIG.OFSH2", signed=False, parent=self)
        self.OFS_READY2     = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "ADC.CSA2_CONFIG.OFS_READY2", signed=False, parent=self)
        self.OFS_READY_FRC2 = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "ADC.CSA2_CONFIG.OFS_READY_FRC2", signed=False, parent=self)
        self.SKIP_CMR2      = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "ADC.CSA2_CONFIG.SKIP_CMR2", signed=False, parent=self)
        self.SKIP_OFS2      = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "ADC.CSA2_CONFIG.SKIP_OFS2", signed=False, parent=self)
        self.OFSCSA_EN2     = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "ADC.CSA2_CONFIG.OFSCSA_EN2", signed=False, parent=self)
        self.OFSADC_EN2     = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "ADC.CSA2_CONFIG.OFSADC_EN2", signed=False, parent=self)
        self.AZ_2           = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "ADC.CSA2_CONFIG.AZ_2", signed=False, parent=self)
        self.AZ2_2          = Field(self.ADDRESS, self.WIDTH, 0x02000000, 25, "ADC.CSA2_CONFIG.AZ2_2", signed=False, parent=self)
        self.AZ3_2          = Field(self.ADDRESS, self.WIDTH, 0x04000000, 26, "ADC.CSA2_CONFIG.AZ3_2", signed=False, parent=self)
        self.AZCM_2         = Field(self.ADDRESS, self.WIDTH, 0x08000000, 27, "ADC.CSA2_CONFIG.AZCM_2", signed=False, parent=self)
        self.CALIB_EN2      = Field(self.ADDRESS, self.WIDTH, 0x10000000, 28, "ADC.CSA2_CONFIG.CALIB_EN2", signed=False, parent=self)
        self.NVALID2        = Field(self.ADDRESS, self.WIDTH, 0x40000000, 30, "ADC.CSA2_CONFIG.NVALID2", signed=False, parent=self)
        self.TRIG2          = Field(self.ADDRESS, self.WIDTH, 0x80000000, 31, "ADC.CSA2_CONFIG.TRIG2", signed=False, parent=self)

class _Adc_Csa3Config(Register):
    ADDRESS = 0x4801C02C
    WIDTH = 4
    DEFAULT = 0x00C80000

    def __init__(self, parent):
        self._PARENT = parent

        self.OFSL3          = Field(self.ADDRESS, self.WIDTH, 0x0000003F,  0, "ADC.CSA3_CONFIG.OFSL3", signed=False, parent=self)
        self.OFSM3          = Field(self.ADDRESS, self.WIDTH, 0x00000FC0,  6, "ADC.CSA3_CONFIG.OFSM3", signed=False, parent=self)
        self.OFSH3          = Field(self.ADDRESS, self.WIDTH, 0x0003F000, 12, "ADC.CSA3_CONFIG.OFSH3", signed=False, parent=self)
        self.OFS_READY3     = Field(self.ADDRESS, self.WIDTH, 0x00040000, 18, "ADC.CSA3_CONFIG.OFS_READY3", signed=False, parent=self)
        self.OFS_READY_FRC3 = Field(self.ADDRESS, self.WIDTH, 0x00080000, 19, "ADC.CSA3_CONFIG.OFS_READY_FRC3", signed=False, parent=self)
        self.SKIP_CMR3      = Field(self.ADDRESS, self.WIDTH, 0x00100000, 20, "ADC.CSA3_CONFIG.SKIP_CMR3", signed=False, parent=self)
        self.SKIP_OFS3      = Field(self.ADDRESS, self.WIDTH, 0x00200000, 21, "ADC.CSA3_CONFIG.SKIP_OFS3", signed=False, parent=self)
        self.OFSCSA_EN3     = Field(self.ADDRESS, self.WIDTH, 0x00400000, 22, "ADC.CSA3_CONFIG.OFSCSA_EN3", signed=False, parent=self)
        self.OFSADC_EN3     = Field(self.ADDRESS, self.WIDTH, 0x00800000, 23, "ADC.CSA3_CONFIG.OFSADC_EN3", signed=False, parent=self)
        self.AZ_3           = Field(self.ADDRESS, self.WIDTH, 0x01000000, 24, "ADC.CSA3_CONFIG.AZ_3", signed=False, parent=self)
        self.AZ2_3          = Field(self.ADDRESS, self.WIDTH, 0x02000000, 25, "ADC.CSA3_CONFIG.AZ2_3", signed=False, parent=self)
        self.AZ3_3          = Field(self.ADDRESS, self.WIDTH, 0x04000000, 26, "ADC.CSA3_CONFIG.AZ3_3", signed=False, parent=self)
        self.AZCM_3         = Field(self.ADDRESS, self.WIDTH, 0x08000000, 27, "ADC.CSA3_CONFIG.AZCM_3", signed=False, parent=self)
        self.CALIB_EN3      = Field(self.ADDRESS, self.WIDTH, 0x10000000, 28, "ADC.CSA3_CONFIG.CALIB_EN3", signed=False, parent=self)
        self.NVALID3        = Field(self.ADDRESS, self.WIDTH, 0x40000000, 30, "ADC.CSA3_CONFIG.NVALID3", signed=False, parent=self)
        self.TRIG3          = Field(self.ADDRESS, self.WIDTH, 0x80000000, 31, "ADC.CSA3_CONFIG.TRIG3", signed=False, parent=self)

class _Adc_Csa10CmrCfg(Register):
    ADDRESS = 0x4801C030
    WIDTH = 4
    DEFAULT = 0x80808080

    def __init__(self, parent):
        self._PARENT = parent

        self.CMRN_DAC0 = Field(self.ADDRESS, self.WIDTH, 0x000000FF,  0, "ADC.CSA1_0_CMR_CFG.CMRN_DAC0", signed=False, parent=self)
        self.CMRP_DAC0 = Field(self.ADDRESS, self.WIDTH, 0x0000FF00,  8, "ADC.CSA1_0_CMR_CFG.CMRP_DAC0", signed=False, parent=self)
        self.CMRN_DAC1 = Field(self.ADDRESS, self.WIDTH, 0x00FF0000, 16, "ADC.CSA1_0_CMR_CFG.CMRN_DAC1", signed=False, parent=self)
        self.CMRP_DAC1 = Field(self.ADDRESS, self.WIDTH, 0xFF000000, 24, "ADC.CSA1_0_CMR_CFG.CMRP_DAC1", signed=False, parent=self)

class _Adc_Csa32CmrCfg(Register):
    ADDRESS = 0x4801C034
    WIDTH = 4
    DEFAULT = 0x80808080

    def __init__(self, parent):
        self._PARENT = parent

        self.CMRN_DAC2 = Field(self.ADDRESS, self.WIDTH, 0x000000FF,  0, "ADC.CSA3_2_CMR_CFG.CMRN_DAC2", signed=False, parent=self)
        self.CMRP_DAC2 = Field(self.ADDRESS, self.WIDTH, 0x0000FF00,  8, "ADC.CSA3_2_CMR_CFG.CMRP_DAC2", signed=False, parent=self)
        self.CMRN_DAC3 = Field(self.ADDRESS, self.WIDTH, 0x00FF0000, 16, "ADC.CSA3_2_CMR_CFG.CMRN_DAC3", signed=False, parent=self)
        self.CMRP_DAC3 = Field(self.ADDRESS, self.WIDTH, 0xFF000000, 24, "ADC.CSA3_2_CMR_CFG.CMRP_DAC3", signed=False, parent=self)


class _OtpCtrl(AddressBlock):
    _NAME       = "OTPCTRL"
    _MEMORY_MAP = "PERIPHERAL_REGISTERS"

    def __init__(self):
        self.SLP_CONTROL    = _OtpCtrl_SlpControl(self)
        self.SLP_MR         = _OtpCtrl_SlpMr(self)
        self.SLP_MREF       = _OtpCtrl_SlpMref(self)
        self.SLP_ADDR       = _OtpCtrl_SlpAddr(self)
        self.SLP_WDB        = _OtpCtrl_SlpWdb(self)
        self.SLP_WDT        = _OtpCtrl_SlpWdt(self)
        self.SLP_RD         = _OtpCtrl_SlpRd(self)
        self.SLP_DEBUG      = _OtpCtrl_SlpDebug(self)
        self.SLP_BURN_TIME  = _OtpCtrl_SlpBurnTime(self)
        self.MTP_CONTROL    = _OtpCtrl_MtpControl(self)
        self.MTP_STATUS     = _OtpCtrl_MtpStatus(self)
        self.MTP_PROT_ADDR  = _OtpCtrl_MtpProtAddr(self)
        self.MTP_PROT_WDATA = _OtpCtrl_MtpProtWdata(self)
        self.MTP_PROT_RDATA = _OtpCtrl_MtpProtRdata(self)
        self.MTP_LEVEL      = _OtpCtrl_MtpLevel(self)
        self.MTP_MRB        = _OtpCtrl_MtpMrb(self)
        self.MTP_MREFB      = _OtpCtrl_MtpMrefb(self)
        self.MTP_MRV        = _OtpCtrl_MtpMrv(self)
        self.MTP_MREFV      = _OtpCtrl_MtpMrefv(self)
        self.CP_CONTROL_1   = _OtpCtrl_CpControl1(self)
        self.CP_CONTROL_2   = _OtpCtrl_CpControl2(self)
        self.CP_STATUS      = _OtpCtrl_CpStatus(self)
        self.CLK_CONFIG1    = _OtpCtrl_ClkConfig1(self)
        self.CLK_CONFIG2    = _OtpCtrl_ClkConfig2(self)
        self.SLP_STATUS     = _OtpCtrl_SlpStatus(self)
        self.MTP_DATA_0     = _OtpCtrl_MtpData0(self)
        self.MTP_DATA_1     = _OtpCtrl_MtpData1(self)
        self.MTP_DATA_2     = _OtpCtrl_MtpData2(self)
        self.MTP_DATA_3     = _OtpCtrl_MtpData3(self)
        self.MTP_DATA_4     = _OtpCtrl_MtpData4(self)
        self.MTP_DATA_5     = _OtpCtrl_MtpData5(self)
        self.MTP_DATA_6     = _OtpCtrl_MtpData6(self)
        self.MTP_DATA_7     = _OtpCtrl_MtpData7(self)
        self.MTP_DATA_8     = _OtpCtrl_MtpData8(self)
        self.MTP_DATA_9     = _OtpCtrl_MtpData9(self)
        self.MTP_DATA_10    = _OtpCtrl_MtpData10(self)
        self.MTP_DATA_11    = _OtpCtrl_MtpData11(self)
        self.MTP_DATA_12    = _OtpCtrl_MtpData12(self)
        self.MTP_DATA_13    = _OtpCtrl_MtpData13(self)
        self.MTP_DATA_14    = _OtpCtrl_MtpData14(self)
        self.MTP_DATA_15    = _OtpCtrl_MtpData15(self)
        self.MTP_DATA_16    = _OtpCtrl_MtpData16(self)
        self.MTP_DATA_17    = _OtpCtrl_MtpData17(self)
        self.MTP_DATA_18    = _OtpCtrl_MtpData18(self)
        self.MTP_DATA_19    = _OtpCtrl_MtpData19(self)
        self.MTP_DATA_20    = _OtpCtrl_MtpData20(self)
        self.MTP_DATA_21    = _OtpCtrl_MtpData21(self)
        self.MTP_DATA_22    = _OtpCtrl_MtpData22(self)
        self.MTP_DATA_23    = _OtpCtrl_MtpData23(self)
        self.MTP_DATA_24    = _OtpCtrl_MtpData24(self)
        self.MTP_DATA_25    = _OtpCtrl_MtpData25(self)
        self.MTP_DATA_26    = _OtpCtrl_MtpData26(self)
        self.MTP_DATA_27    = _OtpCtrl_MtpData27(self)
        self.MTP_DATA_28    = _OtpCtrl_MtpData28(self)
        self.MTP_DATA_29    = _OtpCtrl_MtpData29(self)
        self.MTP_DATA_30    = _OtpCtrl_MtpData30(self)
        self.MTP_DATA_31    = _OtpCtrl_MtpData31(self)
        self.MTP_ADDR       = _OtpCtrl_MtpAddr(self)


class _OtpCtrl_SlpControl(Register):
    ADDRESS = 0x48020000
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.WE         = Field(self.ADDRESS, self.WIDTH, 0x0001,  0, "OTPCTRL.SLP_CONTROL.WE", signed=False, parent=self)
        self.PROT       = Field(self.ADDRESS, self.WIDTH, 0x0002,  1, "OTPCTRL.SLP_CONTROL.PROT", signed=False, parent=self)
        self.LOAD       = Field(self.ADDRESS, self.WIDTH, 0x0004,  2, "OTPCTRL.SLP_CONTROL.LOAD", signed=False, parent=self)
        self.MASK       = Field(self.ADDRESS, self.WIDTH, 0x0008,  3, "OTPCTRL.SLP_CONTROL.MASK", signed=False, parent=self)
        self.TRIM_VALID = Field(self.ADDRESS, self.WIDTH, 0x0010,  4, "OTPCTRL.SLP_CONTROL.TRIM_VALID", signed=False, parent=self)
        self.WR_ON      = Field(self.ADDRESS, self.WIDTH, 0x0020,  5, "OTPCTRL.SLP_CONTROL.WR_ON", signed=False, parent=self)
        self.VDD_ON     = Field(self.ADDRESS, self.WIDTH, 0x0040,  6, "OTPCTRL.SLP_CONTROL.VDD_ON", signed=False, parent=self)
        self.VPP_ON     = Field(self.ADDRESS, self.WIDTH, 0x0080,  7, "OTPCTRL.SLP_CONTROL.VPP_ON", signed=False, parent=self)

class _OtpCtrl_SlpMr(Register):
    ADDRESS = 0x48020002
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MR = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.SLP_MR.MR", signed=False, parent=self)

class _OtpCtrl_SlpMref(Register):
    ADDRESS = 0x48020004
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MREF = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.SLP_MREF.MREF", signed=False, parent=self)

class _OtpCtrl_SlpAddr(Register):
    ADDRESS = 0x48020006
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.ADDR = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.SLP_ADDR.ADDR", signed=False, parent=self)

class _OtpCtrl_SlpWdb(Register):
    ADDRESS = 0x48020008
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.WDB = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.SLP_WDB.WDB", signed=False, parent=self)

class _OtpCtrl_SlpWdt(Register):
    ADDRESS = 0x4802000A
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.WDT = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.SLP_WDT.WDT", signed=False, parent=self)

class _OtpCtrl_SlpRd(Register):
    ADDRESS = 0x4802000C
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.RDATA = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.SLP_RD.RDATA", signed=False, parent=self)

class _OtpCtrl_SlpDebug(Register):
    ADDRESS = 0x4802000E
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.SLP_RESTORE = Field(self.ADDRESS, self.WIDTH, 0x0001,  0, "OTPCTRL.SLP_DEBUG.SLP_RESTORE", signed=False, parent=self)
        self.ECC_DIS     = Field(self.ADDRESS, self.WIDTH, 0x0002,  1, "OTPCTRL.SLP_DEBUG.ECC_DIS", signed=False, parent=self)
        self.ECC_STATUS  = Field(self.ADDRESS, self.WIDTH, 0x0004,  2, "OTPCTRL.SLP_DEBUG.ECC_STATUS", signed=False, parent=self)

class _OtpCtrl_SlpBurnTime(Register):
    ADDRESS = 0x48020010
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.BURN_TIME = Field(self.ADDRESS, self.WIDTH, 0x00FF,  0, "OTPCTRL.SLP_BURN_TIME.BURN_TIME", signed=False, parent=self)

class _OtpCtrl_MtpControl(Register):
    ADDRESS = 0x48020012
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.SRT_PROG    = Field(self.ADDRESS, self.WIDTH, 0x0001,  0, "OTPCTRL.MTP_CONTROL.SRT_PROG", signed=False, parent=self)
        self.STOP_PROG   = Field(self.ADDRESS, self.WIDTH, 0x0002,  1, "OTPCTRL.MTP_CONTROL.STOP_PROG", signed=False, parent=self)
        self.MTP_PROT_EN = Field(self.ADDRESS, self.WIDTH, 0x0010,  4, "OTPCTRL.MTP_CONTROL.MTP_PROT_EN", signed=False, parent=self)
        self.MTP_RESTORE = Field(self.ADDRESS, self.WIDTH, 0x0080,  7, "OTPCTRL.MTP_CONTROL.MTP_RESTORE", signed=False, parent=self)

class _OtpCtrl_MtpStatus(Register):
    ADDRESS = 0x48020014
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.VERI_FAIL            = Field(self.ADDRESS, self.WIDTH, 0x0002,  1, "OTPCTRL.MTP_STATUS.VERI_FAIL", signed=False, parent=self)
        self.MTP_FULL             = Field(self.ADDRESS, self.WIDTH, 0x0004,  2, "OTPCTRL.MTP_STATUS.MTP_FULL", signed=False, parent=self)
        self.VPP_INIT_FAIL        = Field(self.ADDRESS, self.WIDTH, 0x0008,  3, "OTPCTRL.MTP_STATUS.VPP_INIT_FAIL", signed=False, parent=self)
        self.OV_DURING_BURN_PULSE = Field(self.ADDRESS, self.WIDTH, 0x0010,  4, "OTPCTRL.MTP_STATUS.OV_DURING_BURN_PULSE", signed=False, parent=self)
        self.ECC_ERR_1BIT         = Field(self.ADDRESS, self.WIDTH, 0x0020,  5, "OTPCTRL.MTP_STATUS.ECC_ERR_1BIT", signed=False, parent=self)
        self.ECC_ERR_2BIT         = Field(self.ADDRESS, self.WIDTH, 0x0040,  6, "OTPCTRL.MTP_STATUS.ECC_ERR_2BIT", signed=False, parent=self)
        self.DONE                 = Field(self.ADDRESS, self.WIDTH, 0x0080,  7, "OTPCTRL.MTP_STATUS.DONE", signed=False, parent=self)

class _OtpCtrl_MtpProtAddr(Register):
    ADDRESS = 0x48020016
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_PROT_ADDR = Field(self.ADDRESS, self.WIDTH, 0x00FF,  0, "OTPCTRL.MTP_PROT_ADDR.MTP_PROT_ADDR", signed=False, parent=self)

class _OtpCtrl_MtpProtWdata(Register):
    ADDRESS = 0x48020018
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_PROT_WDATA = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_PROT_WDATA.MTP_PROT_WDATA", signed=False, parent=self)

class _OtpCtrl_MtpProtRdata(Register):
    ADDRESS = 0x4802001A
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_PROT_RDATA = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_PROT_RDATA.MTP_PROT_RDATA", signed=False, parent=self)

class _OtpCtrl_MtpLevel(Register):
    ADDRESS = 0x4802001C
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_LEVEL = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_LEVEL.MTP_LEVEL", signed=False, parent=self)

class _OtpCtrl_MtpMrb(Register):
    ADDRESS = 0x4802001E
    WIDTH = 2
    DEFAULT = 0x0302

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_MRB = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_MRB.MTP_MRB", signed=False, parent=self)

class _OtpCtrl_MtpMrefb(Register):
    ADDRESS = 0x48020020
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_MREFB = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_MREFB.MTP_MREFB", signed=False, parent=self)

class _OtpCtrl_MtpMrv(Register):
    ADDRESS = 0x48020022
    WIDTH = 2
    DEFAULT = 0x1B02

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_MRV = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_MRV.MTP_MRV", signed=False, parent=self)

class _OtpCtrl_MtpMrefv(Register):
    ADDRESS = 0x48020024
    WIDTH = 2
    DEFAULT = 0x0400

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_MREFV = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_MREFV.MTP_MREFV", signed=False, parent=self)

class _OtpCtrl_CpControl1(Register):
    ADDRESS = 0x48020026
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.R_CPCTRL = Field(self.ADDRESS, self.WIDTH, 0x00FF,  0, "OTPCTRL.CP_CONTROL_1.R_CPCTRL", signed=False, parent=self)

class _OtpCtrl_CpControl2(Register):
    ADDRESS = 0x48020028
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.SLP_MAX_STAGES  = Field(self.ADDRESS, self.WIDTH, 0x0003,  0, "OTPCTRL.CP_CONTROL_2.SLP_MAX_STAGES", signed=False, parent=self)
        self.SLP_EN_VRPCP    = Field(self.ADDRESS, self.WIDTH, 0x0004,  2, "OTPCTRL.CP_CONTROL_2.SLP_EN_VRPCP", signed=False, parent=self)
        self.SLP_EN_WINCMP   = Field(self.ADDRESS, self.WIDTH, 0x0008,  3, "OTPCTRL.CP_CONTROL_2.SLP_EN_WINCMP", signed=False, parent=self)
        self.SLP_EN_STG_SEL  = Field(self.ADDRESS, self.WIDTH, 0x0010,  4, "OTPCTRL.CP_CONTROL_2.SLP_EN_STG_SEL", signed=False, parent=self)
        self.SLP_EN_STGO_SEL = Field(self.ADDRESS, self.WIDTH, 0x0020,  5, "OTPCTRL.CP_CONTROL_2.SLP_EN_STGO_SEL", signed=False, parent=self)
        self.UV_DISABLE      = Field(self.ADDRESS, self.WIDTH, 0x0040,  6, "OTPCTRL.CP_CONTROL_2.UV_DISABLE", signed=False, parent=self)
        self.OV_DISABLE      = Field(self.ADDRESS, self.WIDTH, 0x0080,  7, "OTPCTRL.CP_CONTROL_2.OV_DISABLE", signed=False, parent=self)

class _OtpCtrl_CpStatus(Register):
    ADDRESS = 0x4802002A
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.CP_CMPOUT      = Field(self.ADDRESS, self.WIDTH, 0x0007,  0, "OTPCTRL.CP_STATUS.CP_CMPOUT", signed=False, parent=self)
        self.CP_ERR         = Field(self.ADDRESS, self.WIDTH, 0x0008,  3, "OTPCTRL.CP_STATUS.CP_ERR", signed=False, parent=self)
        self.PROG_ERR       = Field(self.ADDRESS, self.WIDTH, 0x0010,  4, "OTPCTRL.CP_STATUS.PROG_ERR", signed=False, parent=self)
        self.SLVHS_OVERRIDE = Field(self.ADDRESS, self.WIDTH, 0x0020,  5, "OTPCTRL.CP_STATUS.SLVHS_OVERRIDE", signed=False, parent=self)
        self.SLVHS_VPPDETON = Field(self.ADDRESS, self.WIDTH, 0x0080,  7, "OTPCTRL.CP_STATUS.SLVHS_VPPDETON", signed=False, parent=self)

class _OtpCtrl_ClkConfig1(Register):
    ADDRESS = 0x4802002C
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.FTP_READ_PULSE_CYCLES_LOW  = Field(self.ADDRESS, self.WIDTH, 0x003F,  0, "OTPCTRL.CLK_CONFIG1.FTP_READ_PULSE_CYCLES_LOW", signed=False, parent=self)
        self.FTP_READ_PULSE_CYCLES_HIGH = Field(self.ADDRESS, self.WIDTH, 0x0FC0,  6, "OTPCTRL.CLK_CONFIG1.FTP_READ_PULSE_CYCLES_HIGH", signed=False, parent=self)

class _OtpCtrl_ClkConfig2(Register):
    ADDRESS = 0x4802002E
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.ROM_CLK_DIV = Field(self.ADDRESS, self.WIDTH, 0x007F,  0, "OTPCTRL.CLK_CONFIG2.ROM_CLK_DIV", signed=False, parent=self)

class _OtpCtrl_SlpStatus(Register):
    ADDRESS = 0x48020030
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.OTP_RESTORED     = Field(self.ADDRESS, self.WIDTH, 0x0001,  0, "OTPCTRL.SLP_STATUS.OTP_RESTORED", signed=False, parent=self)
        self.MTP_RECORD_VALID = Field(self.ADDRESS, self.WIDTH, 0x0002,  1, "OTPCTRL.SLP_STATUS.MTP_RECORD_VALID", signed=False, parent=self)

class _OtpCtrl_MtpData0(Register):
    ADDRESS = 0x48020034
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_0.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData1(Register):
    ADDRESS = 0x48020036
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_1.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData2(Register):
    ADDRESS = 0x48020038
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_2.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData3(Register):
    ADDRESS = 0x4802003A
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_3.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData4(Register):
    ADDRESS = 0x4802003C
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_4.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData5(Register):
    ADDRESS = 0x4802003E
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_5.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData6(Register):
    ADDRESS = 0x48020040
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_6.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData7(Register):
    ADDRESS = 0x48020042
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_7.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData8(Register):
    ADDRESS = 0x48020044
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_8.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData9(Register):
    ADDRESS = 0x48020046
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_9.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData10(Register):
    ADDRESS = 0x48020048
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_10.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData11(Register):
    ADDRESS = 0x4802004A
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_11.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData12(Register):
    ADDRESS = 0x4802004C
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_12.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData13(Register):
    ADDRESS = 0x4802004E
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_13.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData14(Register):
    ADDRESS = 0x48020050
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_14.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData15(Register):
    ADDRESS = 0x48020052
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_15.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData16(Register):
    ADDRESS = 0x48020054
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_16.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData17(Register):
    ADDRESS = 0x48020056
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_17.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData18(Register):
    ADDRESS = 0x48020058
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_18.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData19(Register):
    ADDRESS = 0x4802005A
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_19.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData20(Register):
    ADDRESS = 0x4802005C
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_20.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData21(Register):
    ADDRESS = 0x4802005E
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_21.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData22(Register):
    ADDRESS = 0x48020060
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_22.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData23(Register):
    ADDRESS = 0x48020062
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_23.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData24(Register):
    ADDRESS = 0x48020064
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_24.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData25(Register):
    ADDRESS = 0x48020066
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_25.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData26(Register):
    ADDRESS = 0x48020068
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_26.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData27(Register):
    ADDRESS = 0x4802006A
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_27.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData28(Register):
    ADDRESS = 0x4802006C
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_28.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData29(Register):
    ADDRESS = 0x4802006E
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_29.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData30(Register):
    ADDRESS = 0x48020070
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_30.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpData31(Register):
    ADDRESS = 0x48020072
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MTP_RECORD = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "OTPCTRL.MTP_DATA_31.MTP_RECORD", signed=False, parent=self)

class _OtpCtrl_MtpAddr(Register):
    ADDRESS = 0x48020074
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.ADDR = Field(self.ADDRESS, self.WIDTH, 0x0007,  0, "OTPCTRL.MTP_ADDR.ADDR", signed=False, parent=self)


class _Config(AddressBlock):
    _NAME       = "CONFIG"
    _MEMORY_MAP = "OTP_BOOT_CONFIG"

    def __init__(self):
        self.BOOT_00_POWER                   = _Config_Boot00Power(self)
        self.BOOT_01_ADDRESS                 = _Config_Boot01Address(self)
        self.BOOT_02_UART_TXEN_DELAY         = _Config_Boot02UartTxenDelay(self)
        self.BOOT_03_BOOT_INTERFACE          = _Config_Boot03BootInterface(self)
        self.BOOT_04_BOOTSTRAP               = _Config_Boot04Bootstrap(self)
        self.BOOT_05_FLASH                   = _Config_Boot05Flash(self)
        self.BOOT_06_EEPROM                  = _Config_Boot06Eeprom(self)
        self.BOOT_07_GPIO_DATA_0_15_INIT     = _Config_Boot07GpioData015Init(self)
        self.BOOT_08_GPIO_DIR_0_15_INIT      = _Config_Boot08GpioDir015Init(self)
        self.BOOT_09_GPIO_PULLUP_0_15_INIT   = _Config_Boot09GpioPullup015Init(self)
        self.BOOT_0A_GPIO_PULLDOWN_0_15_INIT = _Config_Boot0aGpioPulldown015Init(self)
        self.BOOT_0B_GPIO_16_18_INIT         = _Config_Boot0bGpio1618Init(self)
        self.BOOT_0C_CLK_SEL_INIT0           = _Config_Boot0cClkSelInit0(self)
        self.BOOT_0D_CLK_SEL_INIT1           = _Config_Boot0dClkSelInit1(self)
        self.BOOT_0E_PLL_CONFIG0             = _Config_Boot0ePllConfig0(self)
        self.BOOT_0F_PLL_CONFIG1             = _Config_Boot0fPllConfig1(self)
        self.BOOT_10_APP_CONFIG_0            = _Config_Boot10AppConfig0(self)
        self.BOOT_11_APP_CONFIG_1            = _Config_Boot11AppConfig1(self)
        self.BOOT_12_APP_CONFIG_2            = _Config_Boot12AppConfig2(self)
        self.BOOT_13_APP_CONFIG_3            = _Config_Boot13AppConfig3(self)
        self.BOOT_14_APP_CONFIG_4            = _Config_Boot14AppConfig4(self)
        self.BOOT_15_RESERVED                = _Config_Boot15Reserved(self)
        self.BOOT_16_RESERVED                = _Config_Boot16Reserved(self)
        self.BOOT_17_RESERVED                = _Config_Boot17Reserved(self)
        self.BOOT_18_RESERVED                = _Config_Boot18Reserved(self)
        self.BOOT_19_RESERVED                = _Config_Boot19Reserved(self)
        self.BOOT_1A_RESERVED                = _Config_Boot1aReserved(self)
        self.BOOT_1B_RESERVED                = _Config_Boot1bReserved(self)
        self.BOOT_1C_RESERVED                = _Config_Boot1cReserved(self)
        self.BOOT_1D_RESERVED                = _Config_Boot1dReserved(self)
        self.BOOT_1E_RESERVED                = _Config_Boot1eReserved(self)
        self.BOOT_1F_RESERVED                = _Config_Boot1fReserved(self)


class _Config_Boot00Power(Register):
    ADDRESS = 0x00000000
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.VEXT1           = Field(self.ADDRESS, self.WIDTH, 0x0003,  0, "CONFIG.BOOT_00_POWER.VEXT1", signed=False, parent=self)
        self.VEXT2           = Field(self.ADDRESS, self.WIDTH, 0x000C,  2, "CONFIG.BOOT_00_POWER.VEXT2", signed=False, parent=self)
        self.SS_VEXT1        = Field(self.ADDRESS, self.WIDTH, 0x0030,  4, "CONFIG.BOOT_00_POWER.SS_VEXT1", signed=False, parent=self)
        self.SS_VEXT2        = Field(self.ADDRESS, self.WIDTH, 0x00C0,  6, "CONFIG.BOOT_00_POWER.SS_VEXT2", signed=False, parent=self)
        self.LDO_SHORT_FAULT = Field(self.ADDRESS, self.WIDTH, 0x0100,  8, "CONFIG.BOOT_00_POWER.LDO_SHORT_FAULT", signed=False, parent=self)

class _Config_Boot01Address(Register):
    ADDRESS = 0x00000002
    WIDTH = 2
    DEFAULT = 0xFF01

    def __init__(self, parent):
        self._PARENT = parent

        self.DEVICE_ADDRESS = Field(self.ADDRESS, self.WIDTH, 0x00FF,  0, "CONFIG.BOOT_01_ADDRESS.DEVICE_ADDRESS", signed=False, parent=self)
        self.MASTER_ADDRESS = Field(self.ADDRESS, self.WIDTH, 0xFF00,  8, "CONFIG.BOOT_01_ADDRESS.MASTER_ADDRESS", signed=False, parent=self)

class _Config_Boot02UartTxenDelay(Register):
    ADDRESS = 0x00000004
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.UART_TXEN_POST_DELAY = Field(self.ADDRESS, self.WIDTH, 0x00FF,  0, "CONFIG.BOOT_02_UART_TXEN_DELAY.UART_TXEN_POST_DELAY", signed=False, parent=self)
        self.UART_TXEN_PRE_DELAY  = Field(self.ADDRESS, self.WIDTH, 0xFF00,  8, "CONFIG.BOOT_02_UART_TXEN_DELAY.UART_TXEN_PRE_DELAY", signed=False, parent=self)

class _Config_Boot03BootInterface(Register):
    ADDRESS = 0x00000006
    WIDTH = 2
    DEFAULT = 0x0780

    def __init__(self, parent):
        self._PARENT = parent

        self.BL_DISABLE_UART  = Field(self.ADDRESS, self.WIDTH, 0x0001,  0, "CONFIG.BOOT_03_BOOT_INTERFACE.BL_DISABLE_UART", signed=False, parent=self)
        self.BL_DISABLE_SPI   = Field(self.ADDRESS, self.WIDTH, 0x0002,  1, "CONFIG.BOOT_03_BOOT_INTERFACE.BL_DISABLE_SPI", signed=False, parent=self)
        self.BL_SPI_SELECT    = Field(self.ADDRESS, self.WIDTH, 0x0004,  2, "CONFIG.BOOT_03_BOOT_INTERFACE.BL_SPI_SELECT", signed=False, parent=self)
        self.BL_UART_RX       = Field(self.ADDRESS, self.WIDTH, 0x0008,  3, "CONFIG.BOOT_03_BOOT_INTERFACE.BL_UART_RX", signed=False, parent=self)
        self.BL_UART_TX       = Field(self.ADDRESS, self.WIDTH, 0x0010,  4, "CONFIG.BOOT_03_BOOT_INTERFACE.BL_UART_TX", signed=False, parent=self)
        self.BL_UART_TXEN     = Field(self.ADDRESS, self.WIDTH, 0x0060,  5, "CONFIG.BOOT_03_BOOT_INTERFACE.BL_UART_TXEN", signed=False, parent=self)
        self.BL_UART_BAUDRATE = Field(self.ADDRESS, self.WIDTH, 0x0380,  7, "CONFIG.BOOT_03_BOOT_INTERFACE.BL_UART_BAUDRATE", signed=False, parent=self)
        self.BL_SPI0_SCK      = Field(self.ADDRESS, self.WIDTH, 0x0400, 10, "CONFIG.BOOT_03_BOOT_INTERFACE.BL_SPI0_SCK", signed=False, parent=self)

class _Config_Boot04Bootstrap(Register):
    ADDRESS = 0x00000008
    WIDTH = 2
    DEFAULT = 0x0008

    def __init__(self, parent):
        self._PARENT = parent

        self.BOOT_APP          = Field(self.ADDRESS, self.WIDTH, 0x0003,  0, "CONFIG.BOOT_04_BOOTSTRAP.BOOT_APP", signed=False, parent=self)
        self.BL_ENTRY_FAULT    = Field(self.ADDRESS, self.WIDTH, 0x0004,  2, "CONFIG.BOOT_04_BOOTSTRAP.BL_ENTRY_FAULT", signed=False, parent=self)
        self.BL_EXIT_FAULT     = Field(self.ADDRESS, self.WIDTH, 0x0008,  3, "CONFIG.BOOT_04_BOOTSTRAP.BL_EXIT_FAULT", signed=False, parent=self)
        self.EXT_MEM_RETRIES   = Field(self.ADDRESS, self.WIDTH, 0x00F0,  4, "CONFIG.BOOT_04_BOOTSTRAP.EXT_MEM_RETRIES", signed=False, parent=self)
        self.DISABLE_SELF_TEST = Field(self.ADDRESS, self.WIDTH, 0x0100,  8, "CONFIG.BOOT_04_BOOTSTRAP.DISABLE_SELF_TEST", signed=False, parent=self)
        self.BL_CONFIG_FAULT   = Field(self.ADDRESS, self.WIDTH, 0x0200,  9, "CONFIG.BOOT_04_BOOTSTRAP.BL_CONFIG_FAULT", signed=False, parent=self)
        self.LOAD_ROM_CODE     = Field(self.ADDRESS, self.WIDTH, 0x1000, 12, "CONFIG.BOOT_04_BOOTSTRAP.LOAD_ROM_CODE", signed=False, parent=self)
        self.LOAD_I2C_CODE     = Field(self.ADDRESS, self.WIDTH, 0x2000, 13, "CONFIG.BOOT_04_BOOTSTRAP.LOAD_I2C_CODE", signed=False, parent=self)
        self.LOAD_SPI_CODE     = Field(self.ADDRESS, self.WIDTH, 0x4000, 14, "CONFIG.BOOT_04_BOOTSTRAP.LOAD_SPI_CODE", signed=False, parent=self)
        self.LOAD_OTP_CODE     = Field(self.ADDRESS, self.WIDTH, 0x8000, 15, "CONFIG.BOOT_04_BOOTSTRAP.LOAD_OTP_CODE", signed=False, parent=self)

class _Config_Boot05Flash(Register):
    ADDRESS = 0x0000000A
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.SPI_FLASH_EN    = Field(self.ADDRESS, self.WIDTH, 0x0001,  0, "CONFIG.BOOT_05_FLASH.SPI_FLASH_EN", signed=False, parent=self)
        self.FLASH_SPI_CS_SW = Field(self.ADDRESS, self.WIDTH, 0x00F8,  3, "CONFIG.BOOT_05_FLASH.FLASH_SPI_CS_SW", signed=False, parent=self)
        self.FLASH_SPI_DIV   = Field(self.ADDRESS, self.WIDTH, 0x0F00,  8, "CONFIG.BOOT_05_FLASH.FLASH_SPI_DIV", signed=False, parent=self)

class _Config_Boot06Eeprom(Register):
    ADDRESS = 0x0000000C
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.I2C_EEPROM_EN   = Field(self.ADDRESS, self.WIDTH, 0x0001,  0, "CONFIG.BOOT_06_EEPROM.I2C_EEPROM_EN", signed=False, parent=self)
        self.EEPROM_I2C_SDA  = Field(self.ADDRESS, self.WIDTH, 0x0006,  1, "CONFIG.BOOT_06_EEPROM.EEPROM_I2C_SDA", signed=False, parent=self)
        self.EEPROM_I2C_SCL  = Field(self.ADDRESS, self.WIDTH, 0x0018,  3, "CONFIG.BOOT_06_EEPROM.EEPROM_I2C_SCL", signed=False, parent=self)
        self.EEPROM_I2C_ADDR = Field(self.ADDRESS, self.WIDTH, 0x00E0,  5, "CONFIG.BOOT_06_EEPROM.EEPROM_I2C_ADDR", signed=False, parent=self)
        self.EEPROM_I2C_MUL  = Field(self.ADDRESS, self.WIDTH, 0x0700,  8, "CONFIG.BOOT_06_EEPROM.EEPROM_I2C_MUL", signed=False, parent=self)

class _Config_Boot07GpioData015Init(Register):
    ADDRESS = 0x0000000E
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.GPIO0_OUT  = Field(self.ADDRESS, self.WIDTH, 0x0001,  0, "CONFIG.BOOT_07_GPIO_DATA_0_15_INIT.GPIO0_OUT", signed=False, parent=self)
        self.GPIO1_OUT  = Field(self.ADDRESS, self.WIDTH, 0x0002,  1, "CONFIG.BOOT_07_GPIO_DATA_0_15_INIT.GPIO1_OUT", signed=False, parent=self)
        self.GPIO2_OUT  = Field(self.ADDRESS, self.WIDTH, 0x0004,  2, "CONFIG.BOOT_07_GPIO_DATA_0_15_INIT.GPIO2_OUT", signed=False, parent=self)
        self.GPIO3_OUT  = Field(self.ADDRESS, self.WIDTH, 0x0008,  3, "CONFIG.BOOT_07_GPIO_DATA_0_15_INIT.GPIO3_OUT", signed=False, parent=self)
        self.GPIO4_OUT  = Field(self.ADDRESS, self.WIDTH, 0x0010,  4, "CONFIG.BOOT_07_GPIO_DATA_0_15_INIT.GPIO4_OUT", signed=False, parent=self)
        self.GPIO5_OUT  = Field(self.ADDRESS, self.WIDTH, 0x0020,  5, "CONFIG.BOOT_07_GPIO_DATA_0_15_INIT.GPIO5_OUT", signed=False, parent=self)
        self.GPIO6_OUT  = Field(self.ADDRESS, self.WIDTH, 0x0040,  6, "CONFIG.BOOT_07_GPIO_DATA_0_15_INIT.GPIO6_OUT", signed=False, parent=self)
        self.GPIO7_OUT  = Field(self.ADDRESS, self.WIDTH, 0x0080,  7, "CONFIG.BOOT_07_GPIO_DATA_0_15_INIT.GPIO7_OUT", signed=False, parent=self)
        self.GPIO8_OUT  = Field(self.ADDRESS, self.WIDTH, 0x0100,  8, "CONFIG.BOOT_07_GPIO_DATA_0_15_INIT.GPIO8_OUT", signed=False, parent=self)
        self.GPIO9_OUT  = Field(self.ADDRESS, self.WIDTH, 0x0200,  9, "CONFIG.BOOT_07_GPIO_DATA_0_15_INIT.GPIO9_OUT", signed=False, parent=self)
        self.GPIO10_OUT = Field(self.ADDRESS, self.WIDTH, 0x0400, 10, "CONFIG.BOOT_07_GPIO_DATA_0_15_INIT.GPIO10_OUT", signed=False, parent=self)
        self.GPIO11_OUT = Field(self.ADDRESS, self.WIDTH, 0x0800, 11, "CONFIG.BOOT_07_GPIO_DATA_0_15_INIT.GPIO11_OUT", signed=False, parent=self)
        self.GPIO12_OUT = Field(self.ADDRESS, self.WIDTH, 0x1000, 12, "CONFIG.BOOT_07_GPIO_DATA_0_15_INIT.GPIO12_OUT", signed=False, parent=self)
        self.GPIO13_OUT = Field(self.ADDRESS, self.WIDTH, 0x2000, 13, "CONFIG.BOOT_07_GPIO_DATA_0_15_INIT.GPIO13_OUT", signed=False, parent=self)
        self.GPIO14_OUT = Field(self.ADDRESS, self.WIDTH, 0x4000, 14, "CONFIG.BOOT_07_GPIO_DATA_0_15_INIT.GPIO14_OUT", signed=False, parent=self)
        self.GPIO15_OUT = Field(self.ADDRESS, self.WIDTH, 0x8000, 15, "CONFIG.BOOT_07_GPIO_DATA_0_15_INIT.GPIO15_OUT", signed=False, parent=self)

class _Config_Boot08GpioDir015Init(Register):
    ADDRESS = 0x00000010
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.GPIO0_OUT_EN  = Field(self.ADDRESS, self.WIDTH, 0x0001,  0, "CONFIG.BOOT_08_GPIO_DIR_0_15_INIT.GPIO0_OUT_EN", signed=False, parent=self)
        self.GPIO1_OUT_EN  = Field(self.ADDRESS, self.WIDTH, 0x0002,  1, "CONFIG.BOOT_08_GPIO_DIR_0_15_INIT.GPIO1_OUT_EN", signed=False, parent=self)
        self.GPIO2_OUT_EN  = Field(self.ADDRESS, self.WIDTH, 0x0004,  2, "CONFIG.BOOT_08_GPIO_DIR_0_15_INIT.GPIO2_OUT_EN", signed=False, parent=self)
        self.GPIO3_OUT_EN  = Field(self.ADDRESS, self.WIDTH, 0x0008,  3, "CONFIG.BOOT_08_GPIO_DIR_0_15_INIT.GPIO3_OUT_EN", signed=False, parent=self)
        self.GPIO4_OUT_EN  = Field(self.ADDRESS, self.WIDTH, 0x0010,  4, "CONFIG.BOOT_08_GPIO_DIR_0_15_INIT.GPIO4_OUT_EN", signed=False, parent=self)
        self.GPIO5_OUT_EN  = Field(self.ADDRESS, self.WIDTH, 0x0020,  5, "CONFIG.BOOT_08_GPIO_DIR_0_15_INIT.GPIO5_OUT_EN", signed=False, parent=self)
        self.GPIO6_OUT_EN  = Field(self.ADDRESS, self.WIDTH, 0x0040,  6, "CONFIG.BOOT_08_GPIO_DIR_0_15_INIT.GPIO6_OUT_EN", signed=False, parent=self)
        self.GPIO7_OUT_EN  = Field(self.ADDRESS, self.WIDTH, 0x0080,  7, "CONFIG.BOOT_08_GPIO_DIR_0_15_INIT.GPIO7_OUT_EN", signed=False, parent=self)
        self.GPIO8_OUT_EN  = Field(self.ADDRESS, self.WIDTH, 0x0100,  8, "CONFIG.BOOT_08_GPIO_DIR_0_15_INIT.GPIO8_OUT_EN", signed=False, parent=self)
        self.GPIO9_OUT_EN  = Field(self.ADDRESS, self.WIDTH, 0x0200,  9, "CONFIG.BOOT_08_GPIO_DIR_0_15_INIT.GPIO9_OUT_EN", signed=False, parent=self)
        self.GPIO10_OUT_EN = Field(self.ADDRESS, self.WIDTH, 0x0400, 10, "CONFIG.BOOT_08_GPIO_DIR_0_15_INIT.GPIO10_OUT_EN", signed=False, parent=self)
        self.GPIO11_OUT_EN = Field(self.ADDRESS, self.WIDTH, 0x0800, 11, "CONFIG.BOOT_08_GPIO_DIR_0_15_INIT.GPIO11_OUT_EN", signed=False, parent=self)
        self.GPIO12_OUT_EN = Field(self.ADDRESS, self.WIDTH, 0x1000, 12, "CONFIG.BOOT_08_GPIO_DIR_0_15_INIT.GPIO12_OUT_EN", signed=False, parent=self)
        self.GPIO13_OUT_EN = Field(self.ADDRESS, self.WIDTH, 0x2000, 13, "CONFIG.BOOT_08_GPIO_DIR_0_15_INIT.GPIO13_OUT_EN", signed=False, parent=self)
        self.GPIO14_OUT_EN = Field(self.ADDRESS, self.WIDTH, 0x4000, 14, "CONFIG.BOOT_08_GPIO_DIR_0_15_INIT.GPIO14_OUT_EN", signed=False, parent=self)
        self.GPIO15_OUT_EN = Field(self.ADDRESS, self.WIDTH, 0x8000, 15, "CONFIG.BOOT_08_GPIO_DIR_0_15_INIT.GPIO15_OUT_EN", signed=False, parent=self)

class _Config_Boot09GpioPullup015Init(Register):
    ADDRESS = 0x00000012
    WIDTH = 2
    DEFAULT = 0xFFC0

    def __init__(self, parent):
        self._PARENT = parent

        self.GPIO0_PU  = Field(self.ADDRESS, self.WIDTH, 0x0001,  0, "CONFIG.BOOT_09_GPIO_PULLUP_0_15_INIT.GPIO0_PU", signed=False, parent=self)
        self.GPIO1_PU  = Field(self.ADDRESS, self.WIDTH, 0x0002,  1, "CONFIG.BOOT_09_GPIO_PULLUP_0_15_INIT.GPIO1_PU", signed=False, parent=self)
        self.GPIO2_PU  = Field(self.ADDRESS, self.WIDTH, 0x0004,  2, "CONFIG.BOOT_09_GPIO_PULLUP_0_15_INIT.GPIO2_PU", signed=False, parent=self)
        self.GPIO3_PU  = Field(self.ADDRESS, self.WIDTH, 0x0008,  3, "CONFIG.BOOT_09_GPIO_PULLUP_0_15_INIT.GPIO3_PU", signed=False, parent=self)
        self.GPIO4_PU  = Field(self.ADDRESS, self.WIDTH, 0x0010,  4, "CONFIG.BOOT_09_GPIO_PULLUP_0_15_INIT.GPIO4_PU", signed=False, parent=self)
        self.GPIO5_PU  = Field(self.ADDRESS, self.WIDTH, 0x0020,  5, "CONFIG.BOOT_09_GPIO_PULLUP_0_15_INIT.GPIO5_PU", signed=False, parent=self)
        self.GPIO6_PU  = Field(self.ADDRESS, self.WIDTH, 0x0040,  6, "CONFIG.BOOT_09_GPIO_PULLUP_0_15_INIT.GPIO6_PU", signed=False, parent=self)
        self.GPIO7_PU  = Field(self.ADDRESS, self.WIDTH, 0x0080,  7, "CONFIG.BOOT_09_GPIO_PULLUP_0_15_INIT.GPIO7_PU", signed=False, parent=self)
        self.GPIO8_PU  = Field(self.ADDRESS, self.WIDTH, 0x0100,  8, "CONFIG.BOOT_09_GPIO_PULLUP_0_15_INIT.GPIO8_PU", signed=False, parent=self)
        self.GPIO9_PU  = Field(self.ADDRESS, self.WIDTH, 0x0200,  9, "CONFIG.BOOT_09_GPIO_PULLUP_0_15_INIT.GPIO9_PU", signed=False, parent=self)
        self.GPIO10_PU = Field(self.ADDRESS, self.WIDTH, 0x0400, 10, "CONFIG.BOOT_09_GPIO_PULLUP_0_15_INIT.GPIO10_PU", signed=False, parent=self)
        self.GPIO11_PU = Field(self.ADDRESS, self.WIDTH, 0x0800, 11, "CONFIG.BOOT_09_GPIO_PULLUP_0_15_INIT.GPIO11_PU", signed=False, parent=self)
        self.GPIO12_PU = Field(self.ADDRESS, self.WIDTH, 0x1000, 12, "CONFIG.BOOT_09_GPIO_PULLUP_0_15_INIT.GPIO12_PU", signed=False, parent=self)
        self.GPIO13_PU = Field(self.ADDRESS, self.WIDTH, 0x2000, 13, "CONFIG.BOOT_09_GPIO_PULLUP_0_15_INIT.GPIO13_PU", signed=False, parent=self)
        self.GPIO14_PU = Field(self.ADDRESS, self.WIDTH, 0x4000, 14, "CONFIG.BOOT_09_GPIO_PULLUP_0_15_INIT.GPIO14_PU", signed=False, parent=self)
        self.GPIO15_PU = Field(self.ADDRESS, self.WIDTH, 0x8000, 15, "CONFIG.BOOT_09_GPIO_PULLUP_0_15_INIT.GPIO15_PU", signed=False, parent=self)

class _Config_Boot0aGpioPulldown015Init(Register):
    ADDRESS = 0x00000014
    WIDTH = 2
    DEFAULT = 0x0003

    def __init__(self, parent):
        self._PARENT = parent

        self.GPIO0_PD  = Field(self.ADDRESS, self.WIDTH, 0x0001,  0, "CONFIG.BOOT_0A_GPIO_PULLDOWN_0_15_INIT.GPIO0_PD", signed=False, parent=self)
        self.GPIO1_PD  = Field(self.ADDRESS, self.WIDTH, 0x0002,  1, "CONFIG.BOOT_0A_GPIO_PULLDOWN_0_15_INIT.GPIO1_PD", signed=False, parent=self)
        self.GPIO2_PD  = Field(self.ADDRESS, self.WIDTH, 0x0004,  2, "CONFIG.BOOT_0A_GPIO_PULLDOWN_0_15_INIT.GPIO2_PD", signed=False, parent=self)
        self.GPIO3_PD  = Field(self.ADDRESS, self.WIDTH, 0x0008,  3, "CONFIG.BOOT_0A_GPIO_PULLDOWN_0_15_INIT.GPIO3_PD", signed=False, parent=self)
        self.GPIO4_PD  = Field(self.ADDRESS, self.WIDTH, 0x0010,  4, "CONFIG.BOOT_0A_GPIO_PULLDOWN_0_15_INIT.GPIO4_PD", signed=False, parent=self)
        self.GPIO5_PD  = Field(self.ADDRESS, self.WIDTH, 0x0020,  5, "CONFIG.BOOT_0A_GPIO_PULLDOWN_0_15_INIT.GPIO5_PD", signed=False, parent=self)
        self.GPIO6_PD  = Field(self.ADDRESS, self.WIDTH, 0x0040,  6, "CONFIG.BOOT_0A_GPIO_PULLDOWN_0_15_INIT.GPIO6_PD", signed=False, parent=self)
        self.GPIO7_PD  = Field(self.ADDRESS, self.WIDTH, 0x0080,  7, "CONFIG.BOOT_0A_GPIO_PULLDOWN_0_15_INIT.GPIO7_PD", signed=False, parent=self)
        self.GPIO8_PD  = Field(self.ADDRESS, self.WIDTH, 0x0100,  8, "CONFIG.BOOT_0A_GPIO_PULLDOWN_0_15_INIT.GPIO8_PD", signed=False, parent=self)
        self.GPIO9_PD  = Field(self.ADDRESS, self.WIDTH, 0x0200,  9, "CONFIG.BOOT_0A_GPIO_PULLDOWN_0_15_INIT.GPIO9_PD", signed=False, parent=self)
        self.GPIO10_PD = Field(self.ADDRESS, self.WIDTH, 0x0400, 10, "CONFIG.BOOT_0A_GPIO_PULLDOWN_0_15_INIT.GPIO10_PD", signed=False, parent=self)
        self.GPIO11_PD = Field(self.ADDRESS, self.WIDTH, 0x0800, 11, "CONFIG.BOOT_0A_GPIO_PULLDOWN_0_15_INIT.GPIO11_PD", signed=False, parent=self)
        self.GPIO12_PD = Field(self.ADDRESS, self.WIDTH, 0x1000, 12, "CONFIG.BOOT_0A_GPIO_PULLDOWN_0_15_INIT.GPIO12_PD", signed=False, parent=self)
        self.GPIO13_PD = Field(self.ADDRESS, self.WIDTH, 0x2000, 13, "CONFIG.BOOT_0A_GPIO_PULLDOWN_0_15_INIT.GPIO13_PD", signed=False, parent=self)
        self.GPIO14_PD = Field(self.ADDRESS, self.WIDTH, 0x4000, 14, "CONFIG.BOOT_0A_GPIO_PULLDOWN_0_15_INIT.GPIO14_PD", signed=False, parent=self)
        self.GPIO15_PD = Field(self.ADDRESS, self.WIDTH, 0x8000, 15, "CONFIG.BOOT_0A_GPIO_PULLDOWN_0_15_INIT.GPIO15_PD", signed=False, parent=self)

class _Config_Boot0bGpio1618Init(Register):
    ADDRESS = 0x00000016
    WIDTH = 2
    DEFAULT = 0x0E00

    def __init__(self, parent):
        self._PARENT = parent

        self.GPIO16_OUT      = Field(self.ADDRESS, self.WIDTH, 0x0001,  0, "CONFIG.BOOT_0B_GPIO_16_18_INIT.GPIO16_OUT", signed=False, parent=self)
        self.GPIO17_OUT      = Field(self.ADDRESS, self.WIDTH, 0x0002,  1, "CONFIG.BOOT_0B_GPIO_16_18_INIT.GPIO17_OUT", signed=False, parent=self)
        self.GPIO18_OUT      = Field(self.ADDRESS, self.WIDTH, 0x0004,  2, "CONFIG.BOOT_0B_GPIO_16_18_INIT.GPIO18_OUT", signed=False, parent=self)
        self.GPIO16_OUT_EN   = Field(self.ADDRESS, self.WIDTH, 0x0008,  3, "CONFIG.BOOT_0B_GPIO_16_18_INIT.GPIO16_OUT_EN", signed=False, parent=self)
        self.GPIO17_OUT_EN   = Field(self.ADDRESS, self.WIDTH, 0x0010,  4, "CONFIG.BOOT_0B_GPIO_16_18_INIT.GPIO17_OUT_EN", signed=False, parent=self)
        self.GPIO18_OUT_EN   = Field(self.ADDRESS, self.WIDTH, 0x0020,  5, "CONFIG.BOOT_0B_GPIO_16_18_INIT.GPIO18_OUT_EN", signed=False, parent=self)
        self.GPIO16_PD       = Field(self.ADDRESS, self.WIDTH, 0x0040,  6, "CONFIG.BOOT_0B_GPIO_16_18_INIT.GPIO16_PD", signed=False, parent=self)
        self.GPIO17_PD       = Field(self.ADDRESS, self.WIDTH, 0x0080,  7, "CONFIG.BOOT_0B_GPIO_16_18_INIT.GPIO17_PD", signed=False, parent=self)
        self.GPIO18_PD       = Field(self.ADDRESS, self.WIDTH, 0x0100,  8, "CONFIG.BOOT_0B_GPIO_16_18_INIT.GPIO18_PD", signed=False, parent=self)
        self.GPIO16_PU       = Field(self.ADDRESS, self.WIDTH, 0x0200,  9, "CONFIG.BOOT_0B_GPIO_16_18_INIT.GPIO16_PU", signed=False, parent=self)
        self.GPIO17_PU       = Field(self.ADDRESS, self.WIDTH, 0x0400, 10, "CONFIG.BOOT_0B_GPIO_16_18_INIT.GPIO17_PU", signed=False, parent=self)
        self.GPIO18_PU       = Field(self.ADDRESS, self.WIDTH, 0x0800, 11, "CONFIG.BOOT_0B_GPIO_16_18_INIT.GPIO18_PU", signed=False, parent=self)
        self.GPIO2_ANALOG_EN = Field(self.ADDRESS, self.WIDTH, 0x1000, 12, "CONFIG.BOOT_0B_GPIO_16_18_INIT.GPIO2_ANALOG_EN", signed=False, parent=self)
        self.GPIO3_ANALOG_EN = Field(self.ADDRESS, self.WIDTH, 0x2000, 13, "CONFIG.BOOT_0B_GPIO_16_18_INIT.GPIO3_ANALOG_EN", signed=False, parent=self)
        self.GPIO4_ANALOG_EN = Field(self.ADDRESS, self.WIDTH, 0x4000, 14, "CONFIG.BOOT_0B_GPIO_16_18_INIT.GPIO4_ANALOG_EN", signed=False, parent=self)
        self.GPIO5_ANALOG_EN = Field(self.ADDRESS, self.WIDTH, 0x8000, 15, "CONFIG.BOOT_0B_GPIO_16_18_INIT.GPIO5_ANALOG_EN", signed=False, parent=self)

class _Config_Boot0cClkSelInit0(Register):
    ADDRESS = 0x00000018
    WIDTH = 2
    DEFAULT = 0x0063

    def __init__(self, parent):
        self._PARENT = parent

        self.PLL_FB_DIV   = Field(self.ADDRESS, self.WIDTH, 0x007F,  0, "CONFIG.BOOT_0C_CLK_SEL_INIT0.PLL_FB_DIV", signed=False, parent=self)
        self.EXT_NOT_XTAL = Field(self.ADDRESS, self.WIDTH, 0x0100,  8, "CONFIG.BOOT_0C_CLK_SEL_INIT0.EXT_NOT_XTAL", signed=False, parent=self)
        self.XTAL_CFG     = Field(self.ADDRESS, self.WIDTH, 0x0E00,  9, "CONFIG.BOOT_0C_CLK_SEL_INIT0.XTAL_CFG", signed=False, parent=self)
        self.XTAL_BOOST   = Field(self.ADDRESS, self.WIDTH, 0x1000, 12, "CONFIG.BOOT_0C_CLK_SEL_INIT0.XTAL_BOOST", signed=False, parent=self)
        self.EXT_NOT_INT  = Field(self.ADDRESS, self.WIDTH, 0x2000, 13, "CONFIG.BOOT_0C_CLK_SEL_INIT0.EXT_NOT_INT", signed=False, parent=self)

class _Config_Boot0dClkSelInit1(Register):
    ADDRESS = 0x0000001A
    WIDTH = 2
    DEFAULT = 0x8039

    def __init__(self, parent):
        self._PARENT = parent

        self.PLL_OUT_SEL     = Field(self.ADDRESS, self.WIDTH, 0x0003,  0, "CONFIG.BOOT_0D_CLK_SEL_INIT1.PLL_OUT_SEL", signed=False, parent=self)
        self.RDIV            = Field(self.ADDRESS, self.WIDTH, 0x007C,  2, "CONFIG.BOOT_0D_CLK_SEL_INIT1.RDIV", signed=False, parent=self)
        self.SYS_CLK_DIV     = Field(self.ADDRESS, self.WIDTH, 0x0180,  7, "CONFIG.BOOT_0D_CLK_SEL_INIT1.SYS_CLK_DIV", signed=False, parent=self)
        self.PLL_STATUS      = Field(self.ADDRESS, self.WIDTH, 0x4000, 14, "CONFIG.BOOT_0D_CLK_SEL_INIT1.PLL_STATUS", signed=False, parent=self)
        self.PLL_CONFIG_BOOT = Field(self.ADDRESS, self.WIDTH, 0x8000, 15, "CONFIG.BOOT_0D_CLK_SEL_INIT1.PLL_CONFIG_BOOT", signed=False, parent=self)

class _Config_Boot0ePllConfig0(Register):
    ADDRESS = 0x0000001C
    WIDTH = 2
    DEFAULT = 0x500C

    def __init__(self, parent):
        self._PARENT = parent

        self.ONE_PER_MIN     = Field(self.ADDRESS, self.WIDTH, 0x001F,  0, "CONFIG.BOOT_0E_PLL_CONFIG0.ONE_PER_MIN", signed=False, parent=self)
        self.PLL_DEL_SEL     = Field(self.ADDRESS, self.WIDTH, 0x0060,  5, "CONFIG.BOOT_0E_PLL_CONFIG0.PLL_DEL_SEL", signed=False, parent=self)
        self.N16_PER_MIN_MSB = Field(self.ADDRESS, self.WIDTH, 0x0080,  7, "CONFIG.BOOT_0E_PLL_CONFIG0.N16_PER_MIN_MSB", signed=False, parent=self)
        self.ONE_PER_MAX     = Field(self.ADDRESS, self.WIDTH, 0x1F00,  8, "CONFIG.BOOT_0E_PLL_CONFIG0.ONE_PER_MAX", signed=False, parent=self)
        self.PLL_ERR_OTP     = Field(self.ADDRESS, self.WIDTH, 0x2000, 13, "CONFIG.BOOT_0E_PLL_CONFIG0.PLL_ERR_OTP", signed=False, parent=self)
        self.SEL_ERR_OTP     = Field(self.ADDRESS, self.WIDTH, 0x4000, 14, "CONFIG.BOOT_0E_PLL_CONFIG0.SEL_ERR_OTP", signed=False, parent=self)
        self.N16_PER_MAX_MSB = Field(self.ADDRESS, self.WIDTH, 0x8000, 15, "CONFIG.BOOT_0E_PLL_CONFIG0.N16_PER_MAX_MSB", signed=False, parent=self)

class _Config_Boot0fPllConfig1(Register):
    ADDRESS = 0x0000001E
    WIDTH = 2
    DEFAULT = 0xFFDF

    def __init__(self, parent):
        self._PARENT = parent

        self.N16_PER_MIN = Field(self.ADDRESS, self.WIDTH, 0x00FF,  0, "CONFIG.BOOT_0F_PLL_CONFIG1.N16_PER_MIN", signed=False, parent=self)
        self.N16_PER_MAX = Field(self.ADDRESS, self.WIDTH, 0xFF00,  8, "CONFIG.BOOT_0F_PLL_CONFIG1.N16_PER_MAX", signed=False, parent=self)

class _Config_Boot10AppConfig0(Register):
    ADDRESS = 0x00000020
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.HALL_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x0001,  0, "CONFIG.BOOT_10_APP_CONFIG_0.HALL_ENABLE", signed=False, parent=self)
        self.ABN1_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x0002,  1, "CONFIG.BOOT_10_APP_CONFIG_0.ABN1_ENABLE", signed=False, parent=self)
        self.HALL_UX     = Field(self.ADDRESS, self.WIDTH, 0x0030,  4, "CONFIG.BOOT_10_APP_CONFIG_0.HALL_UX", signed=False, parent=self)
        self.HALL_V      = Field(self.ADDRESS, self.WIDTH, 0x00C0,  6, "CONFIG.BOOT_10_APP_CONFIG_0.HALL_V", signed=False, parent=self)
        self.HALL_WY     = Field(self.ADDRESS, self.WIDTH, 0x0300,  8, "CONFIG.BOOT_10_APP_CONFIG_0.HALL_WY", signed=False, parent=self)
        self.ABN1_A      = Field(self.ADDRESS, self.WIDTH, 0x0C00, 10, "CONFIG.BOOT_10_APP_CONFIG_0.ABN1_A", signed=False, parent=self)
        self.ABN1_B      = Field(self.ADDRESS, self.WIDTH, 0x3000, 12, "CONFIG.BOOT_10_APP_CONFIG_0.ABN1_B", signed=False, parent=self)
        self.ABN1_N      = Field(self.ADDRESS, self.WIDTH, 0xC000, 14, "CONFIG.BOOT_10_APP_CONFIG_0.ABN1_N", signed=False, parent=self)

class _Config_Boot11AppConfig1(Register):
    ADDRESS = 0x00000022
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.REF_L_PIN      = Field(self.ADDRESS, self.WIDTH, 0x0003,  0, "CONFIG.BOOT_11_APP_CONFIG_1.REF_L_PIN", signed=False, parent=self)
        self.REF_R_PIN      = Field(self.ADDRESS, self.WIDTH, 0x000C,  2, "CONFIG.BOOT_11_APP_CONFIG_1.REF_R_PIN", signed=False, parent=self)
        self.REF_H_PIN      = Field(self.ADDRESS, self.WIDTH, 0x0070,  4, "CONFIG.BOOT_11_APP_CONFIG_1.REF_H_PIN", signed=False, parent=self)
        self.STEPDIR_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x0100,  8, "CONFIG.BOOT_11_APP_CONFIG_1.STEPDIR_ENABLE", signed=False, parent=self)
        self.STEP_PIN       = Field(self.ADDRESS, self.WIDTH, 0x0600,  9, "CONFIG.BOOT_11_APP_CONFIG_1.STEP_PIN", signed=False, parent=self)
        self.DIR_PIN        = Field(self.ADDRESS, self.WIDTH, 0x0800, 11, "CONFIG.BOOT_11_APP_CONFIG_1.DIR_PIN", signed=False, parent=self)
        self.ABN2_ENABLE    = Field(self.ADDRESS, self.WIDTH, 0x1000, 12, "CONFIG.BOOT_11_APP_CONFIG_1.ABN2_ENABLE", signed=False, parent=self)
        self.ABN2_A         = Field(self.ADDRESS, self.WIDTH, 0x2000, 13, "CONFIG.BOOT_11_APP_CONFIG_1.ABN2_A", signed=False, parent=self)
        self.ABN2_B         = Field(self.ADDRESS, self.WIDTH, 0xC000, 14, "CONFIG.BOOT_11_APP_CONFIG_1.ABN2_B", signed=False, parent=self)

class _Config_Boot12AppConfig2(Register):
    ADDRESS = 0x00000024
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.WDG_DISABLE         = Field(self.ADDRESS, self.WIDTH, 0x0001,  0, "CONFIG.BOOT_12_APP_CONFIG_2.WDG_DISABLE", signed=False, parent=self)
        self.WDG_TIMEOUT         = Field(self.ADDRESS, self.WIDTH, 0x000E,  1, "CONFIG.BOOT_12_APP_CONFIG_2.WDG_TIMEOUT", signed=False, parent=self)
        self.BRAKECHOPPER_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x0010,  4, "CONFIG.BOOT_12_APP_CONFIG_2.BRAKECHOPPER_ENABLE", signed=False, parent=self)
        self.BRAKECHOPPER_OUTPUT = Field(self.ADDRESS, self.WIDTH, 0x03E0,  5, "CONFIG.BOOT_12_APP_CONFIG_2.BRAKECHOPPER_OUTPUT", signed=False, parent=self)
        self.MECH_BRAKE_ENABLE   = Field(self.ADDRESS, self.WIDTH, 0x1000, 12, "CONFIG.BOOT_12_APP_CONFIG_2.MECH_BRAKE_ENABLE", signed=False, parent=self)
        self.MECH_BRAKE_OUTPUT   = Field(self.ADDRESS, self.WIDTH, 0x6000, 13, "CONFIG.BOOT_12_APP_CONFIG_2.MECH_BRAKE_OUTPUT", signed=False, parent=self)

class _Config_Boot13AppConfig3(Register):
    ADDRESS = 0x00000026
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.SPI_ENC_ENABLE = Field(self.ADDRESS, self.WIDTH, 0x0001,  0, "CONFIG.BOOT_13_APP_CONFIG_3.SPI_ENC_ENABLE", signed=False, parent=self)
        self.SPI_ENC_BLOCK  = Field(self.ADDRESS, self.WIDTH, 0x0002,  1, "CONFIG.BOOT_13_APP_CONFIG_3.SPI_ENC_BLOCK", signed=False, parent=self)
        self.SPI_ENC_MODE   = Field(self.ADDRESS, self.WIDTH, 0x000C,  2, "CONFIG.BOOT_13_APP_CONFIG_3.SPI_ENC_MODE", signed=False, parent=self)
        self.SPI_ENC_FREQ   = Field(self.ADDRESS, self.WIDTH, 0x00F0,  4, "CONFIG.BOOT_13_APP_CONFIG_3.SPI_ENC_FREQ", signed=False, parent=self)
        self.SPI_ENC_CS_PIN = Field(self.ADDRESS, self.WIDTH, 0x0300,  8, "CONFIG.BOOT_13_APP_CONFIG_3.SPI_ENC_CS_PIN", signed=False, parent=self)
        self.SPI_ENC_CS_POL = Field(self.ADDRESS, self.WIDTH, 0x0400, 10, "CONFIG.BOOT_13_APP_CONFIG_3.SPI_ENC_CS_POL", signed=False, parent=self)

class _Config_Boot14AppConfig4(Register):
    ADDRESS = 0x00000028
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.MEM_TMCL_SCRIPT = Field(self.ADDRESS, self.WIDTH, 0x0003,  0, "CONFIG.BOOT_14_APP_CONFIG_4.MEM_TMCL_SCRIPT", signed=False, parent=self)
        self.MEM_PARAMETERS  = Field(self.ADDRESS, self.WIDTH, 0x000C,  2, "CONFIG.BOOT_14_APP_CONFIG_4.MEM_PARAMETERS", signed=False, parent=self)
        self.MEM_STIMULUS    = Field(self.ADDRESS, self.WIDTH, 0x0010,  4, "CONFIG.BOOT_14_APP_CONFIG_4.MEM_STIMULUS", signed=False, parent=self)

class _Config_Boot15Reserved(Register):
    ADDRESS = 0x0000002A
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.RESERVED = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "CONFIG.BOOT_15_RESERVED.RESERVED", signed=False, parent=self)

class _Config_Boot16Reserved(Register):
    ADDRESS = 0x0000002C
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.RESERVED = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "CONFIG.BOOT_16_RESERVED.RESERVED", signed=False, parent=self)

class _Config_Boot17Reserved(Register):
    ADDRESS = 0x0000002E
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.RESERVED = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "CONFIG.BOOT_17_RESERVED.RESERVED", signed=False, parent=self)

class _Config_Boot18Reserved(Register):
    ADDRESS = 0x00000030
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.RESERVED = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "CONFIG.BOOT_18_RESERVED.RESERVED", signed=False, parent=self)

class _Config_Boot19Reserved(Register):
    ADDRESS = 0x00000032
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.RESERVED = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "CONFIG.BOOT_19_RESERVED.RESERVED", signed=False, parent=self)

class _Config_Boot1aReserved(Register):
    ADDRESS = 0x00000034
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.RESERVED = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "CONFIG.BOOT_1A_RESERVED.RESERVED", signed=False, parent=self)

class _Config_Boot1bReserved(Register):
    ADDRESS = 0x00000036
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.RESERVED = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "CONFIG.BOOT_1B_RESERVED.RESERVED", signed=False, parent=self)

class _Config_Boot1cReserved(Register):
    ADDRESS = 0x00000038
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.RESERVED = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "CONFIG.BOOT_1C_RESERVED.RESERVED", signed=False, parent=self)

class _Config_Boot1dReserved(Register):
    ADDRESS = 0x0000003A
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.RESERVED = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "CONFIG.BOOT_1D_RESERVED.RESERVED", signed=False, parent=self)

class _Config_Boot1eReserved(Register):
    ADDRESS = 0x0000003C
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.RESERVED = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "CONFIG.BOOT_1E_RESERVED.RESERVED", signed=False, parent=self)

class _Config_Boot1fReserved(Register):
    ADDRESS = 0x0000003E
    WIDTH = 2
    DEFAULT = 0x0000

    def __init__(self, parent):
        self._PARENT = parent

        self.RESERVED = Field(self.ADDRESS, self.WIDTH, 0xFFFF,  0, "CONFIG.BOOT_1F_RESERVED.RESERVED", signed=False, parent=self)


TM01 = _tm01()
