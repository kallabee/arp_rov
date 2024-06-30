from abc import ABCMeta
from abc import abstractmethod

import numpy as np

class status_display(metaclass = ABCMeta):
    @abstractmethod
    def draw(img:np.array):
        pass