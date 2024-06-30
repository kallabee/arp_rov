from my_lib import *
import time

if __name__ == "__main__":

    st = status_logger('./log_test_2')
    while(True):
        st.log('hi.')
        time.sleep(1)