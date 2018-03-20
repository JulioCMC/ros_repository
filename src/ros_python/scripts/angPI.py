import numpy as np

def ver(phi):

    n   = np.fix(phi/np.pi)

    imp = (-(np.power((-1),n))+1)/2

    if n>0:

        ver=imp+n

    else:

        ver=n-imp

    ang = phi-(ver*np.pi)

    return ang
