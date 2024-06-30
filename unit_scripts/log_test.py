
import os
from dataclasses import dataclass
import logging
from logging import handlers
import time

if __name__ == "__main__":

    formatter = logging.Formatter("%(message)s")

    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG)

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)

    while(True):
        logger.info('hi.')
        time.sleep(1)