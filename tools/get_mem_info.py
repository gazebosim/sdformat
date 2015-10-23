#!/usr/bin/env python

import os;
import psutil;
# get pid of parent process
pid = os.getppid();
# get and print memory usage of parent process
p = psutil.Process(pid);
# note that different versions of psutil have different API
# some versions have memory_info(), others have get_memory_info()
memory_info = (p.memory_info() if hasattr(p, 'memory_info') else p.get_memory_info());
print memory_info[0]
