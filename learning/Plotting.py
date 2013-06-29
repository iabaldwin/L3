class Plotting:

    def __init__(self):
        pass

    @staticmethod
    def plot( dataset ):

        poses = dataset.INS_data

        X = []
        Y = []

        for pose in poses:
            X.append( pose.x )
            Y.append( pose.y )
        
        pyplot.scatter( X, Y )
        pyplot.show()

    
    @staticmethod
    def plotCloud( cloud ):

        pyplot.scatter( X, Y )
        pyplot.show()

