from xmera.architecture import sim_model

class TickerModule(sim_model.SysModel):

    def __init__(self):
        super(TickerModule, self).__init__()

        self._ticker = 0
        self.moduleTag = "testModule"

    def updateState(self, current_sim_nanos):
        self._ticker += 1

    def GetTicker(self):
        return self._ticker

    def reset(self, current_sim_nanos):
        pass
