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


    @staticmethod
    def listExperiences():
        datasets = Listing.listDatasets()

        experiences = []

        for dataset in datasets:

            if os.path.exists( os.path.join( dataset, 'L3', 'experience.dat' ) ):
                experiences.append( dataset )


        return experiences


