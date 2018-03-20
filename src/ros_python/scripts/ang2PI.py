import numpy as np

def ver(phi,phia):

    a = phia-phi

    if a>0 and a>1 and phi<0 and phia>0:

        ang = phia - 2*np.pi

        return ang

    else:

        if (a<0 and a<-1 and phi>0 and phia<0):

            ang = phia + 2*np.pi

            return ang

        else:

            ang = phia

            return ang

        
