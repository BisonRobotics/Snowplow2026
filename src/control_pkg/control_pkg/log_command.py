from control_pkg.commands import Command
from rclpy.impl.rcutils_logger import RcutilsLogger

class LoggingCommand(Command):
    """
    Command that turns the pivot to the given degrees
    """
    def __init__(self, message: str, logger: RcutilsLogger):
        super().__init__()
        self.message = message
        self.logger = logger
                
    def execute(self):
        self.logger.info(self.message)
            
    def is_finished(self) -> bool:
        return True