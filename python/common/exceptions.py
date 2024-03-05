class Lite6Error(Exception):
    pass


class Lite6PliantError(Lite6Error):
    pass


class Lite6SimulationPliant(Lite6PliantError):
    pass


class Lite6HardwarePliant(Lite6PliantError):
    pass


class Lite6PliantChoreographerError(Lite6PliantError):
    pass


class Lite6SystemError(Lite6Error):
    pass
