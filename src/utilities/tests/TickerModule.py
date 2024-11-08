from Basilisk.architecture import sysModel
class TickerModule(sysModel.SysModel):

    def __init__(self):
        super(TickerModule, self).__init__()

        self._ticker = 0
        self.moduleTag = "testModule"

    def updateState(self, current_sim_nanos):
        self._ticker += 1

    def GetTicker(self):
        return self._ticker

    def Reset(self, current_sim_nanos):
        pass
