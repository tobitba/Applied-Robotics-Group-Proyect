from smc import load_config
from smc.logging.logger import Logger

if __name__ == "__main__":
    cfg = load_config()
    log_manager = Logger(None)
    log_manager.loadLog(cfg.load_log_file)
    # TODO: put this into tabs
    log_manager.plotAllControlLoops()
