#!/usr/bin/env python

import os;
import psutil;
pid = os.getppid();
p = psutil.Process(pid);
parent = (p.parent() if callable(p.parent) else p.parent);
memory_info = (parent.memory_info() if hasattr(parent, 'memory_info') else parent.get_memory_info());
print memory_info[0]
