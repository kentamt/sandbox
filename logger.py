import logging
from logging import getLogger, StreamHandler, FileHandler, DEBUG, INFO

logger = getLogger(__name__)
st_handler = StreamHandler()

logger.setLevel(DEBUG)
st_handler.setLevel(DEBUG)

# FORMAT = "[%(asctime)2s:  %(filename)-5s:%(lineno)-5s %(funcName)10s()] [ %(levelname)s ] %(message)s"
# FORMAT = "[%(asctime)2s:  %(filename)-20s:%(lineno)-5s] [%(levelname)s] %(message)s"
FORMAT = "[%(filename)-20s:%(lineno)-5s] [%(levelname)s] %(message)s"
formatter = logging.Formatter(FORMAT)
st_handler.setFormatter(formatter)
logger.addHandler(st_handler)

logger.propagate = False
