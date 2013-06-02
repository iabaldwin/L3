import os

class Listing:

    @staticmethod
    def listDatasets():
        datasets = []
        for a,b,c in os.walk( os.path.expanduser( '~/code/datasets/' ) ):

            split = a.split('/')

            if split[-1] == 'L3':
                datasets.append( '/'.join( split[0:-1] ))
                
        return datasets



