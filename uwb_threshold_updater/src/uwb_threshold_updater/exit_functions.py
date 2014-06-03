#!/usr/bin/env python 


def save_gamma_list(gamma_list, fname="gammaList.txt"):
    '''
        Saves the gamma list as space sperated text file 

        returns if the gamma list is empty
    ''' 
    if len(gamma_list) == 0:
        return
    with open(fname, 'w') as f:
        for gamma in gamma_list:
            f.write(str(gamma) + ' ')

    







